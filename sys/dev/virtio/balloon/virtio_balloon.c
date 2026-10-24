/* https://docs.oasis-open.org/virtio/virtio/v1.1/csprd01/virtio-v1.1-csprd01.html */
/* https://docs.oasis-open.org/virtio/virtio/v1.4/virtio-v1.4.html#x1-4490001 */
/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2011, Bryan Venteicher <bryanv@FreeBSD.org>
 * Copyright (c) 2026, Dennis Camera <dennis.camera+freebsd@riiengineering.ch>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// TODO:
//  - mutex, locks, stuff
//  - huge table stats?
//  - multiple interrupts

/* Driver for VirtIO memory balloon devices. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sglist.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/taskqueue.h>
#include <sys/queue.h>

#include <vm/vm.h>
#include <vm/vm_page.h>
#include <sys/vmmeter.h>
#include <vm/vm_extern.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/virtio/virtio.h>
#include <dev/virtio/virtio_endian.h>
#include <dev/virtio/virtqueue.h>
#include <dev/virtio/balloon/virtio_balloon.h>

#include "virtio_if.h"

struct vtballoon_softc {
	device_t		 vtballoon_dev;
	struct mtx		 vtballoon_mtx;
	uint64_t		 vtballoon_features;
	uint32_t		 vtballoon_flags;
#define VTBALLOON_FLAG_DETACH	 0x01

	uint8_t                  vtballoon_poison_val[4];

	struct virtqueue	*vtballoon_inflate_vq;
	struct virtqueue	*vtballoon_deflate_vq;
	struct virtqueue	*vtballoon_stats_vq;
	struct virtqueue	*vtballoon_free_page_vq;
	struct virtqueue	*vtballoon_reporting_vq;

	struct taskqueue        *vtballoon_tq;
	struct task              vtballoon_configchanged_task;
	struct task              vtballoon_deflate_response_task;
	struct task              vtballoon_inflate_response_task;
	struct task              vtballoon_stats_request_task;

	uint32_t		 vtballoon_desired_npages;
	uint32_t		 vtballoon_current_npages;
	TAILQ_HEAD(,vm_page)	 vtballoon_pages;
	TAILQ_HEAD(,vm_page)	 vtballoon_pages_deflating;

	int			 vtballoon_timeout;

	struct virtio_balloon_stat vtballoon_stats[VIRTIO_BALLOON_S_NR];  /* little endian */
};

static struct virtio_feature_desc vtballoon_feature_desc[] = {
	{ VIRTIO_BALLOON_F_MUST_TELL_HOST,	"MustTellHost"	},
	{ VIRTIO_BALLOON_F_STATS_VQ,		"Stats"         },
	{ VIRTIO_BALLOON_F_DEFLATE_ON_OOM,	"DeflateOnOOM"	},
	{ VIRTIO_BALLOON_F_PAGE_POISON,         "PagePoison"    },
	{ VIRTIO_BALLOON_F_PAGE_REPORTING,      "PageReporting" },
	{ VIRTIO_BALLOON_F_FREE_PAGE_HINT,      "FreePageHint"  },
	{ 0, NULL }
};

struct vtballoon_page_frames {
	int npages;
	void *buf;  // buffer passed to device
	vm_page_t pages[];
};

static int	vtballoon_probe(device_t);
static int	vtballoon_attach(device_t);
static int	vtballoon_detach(device_t);
static uint32_t	vtballoon_desired_size(struct vtballoon_softc *sc);
static int	vtballoon_config_change(device_t);

static void	vtballoon_start_taskqueues(struct vtballoon_softc *);
static void	vtballoon_free_taskqueues(struct vtballoon_softc *);
static void	vtballoon_drain_taskqueues(struct vtballoon_softc *);

static int	vtballoon_negotiate_features(struct vtballoon_softc *);
static int	vtballoon_setup_features(struct vtballoon_softc *);
static int	vtballoon_alloc_virtqueues(struct vtballoon_softc *);

static void	vtballoon_vq_intr(void *);

static void	vtballoon_configchanged_task(void *, int);

static void	vtballoon_stats_request_task(void *, int);
static size_t	vtballoon_update_stats(struct vtballoon_softc *);
static int	vtballoon_send_stats(struct vtballoon_softc *);

static void	vtballoon_inflate(struct vtballoon_softc *, int);
static void	vtballoon_inflate_response_task(void *, int);
static void	vtballoon_deflate(struct vtballoon_softc *, int);
static void	vtballoon_deflate_response_task(void *, int);

static void	vtballoon_adjust_size(struct vtballoon_softc *sc);

static void	vtballoon_pop(struct vtballoon_softc *);
static void	vtballoon_stop(struct vtballoon_softc *);

static vm_page_t
		vtballoon_alloc_page(struct vtballoon_softc *);
static void	vtballoon_free_page(struct vtballoon_softc *, vm_page_t);

static void	vtballoon_setup_sysctl(struct vtballoon_softc *);

#define vtballoon_modern(_sc) \
    (((_sc)->vtballoon_features & VIRTIO_F_VERSION_1) != 0)

/* Features desired/implemented by this driver. */
#define VTBALLOON_FEATURES		(VIRTIO_BALLOON_F_MUST_TELL_HOST | VIRTIO_BALLOON_F_STATS_VQ)

/* Timeout between retries when the balloon needs inflating. */
#define VTBALLOON_LOWMEM_TIMEOUT	hz

/*
 * Maximum number of pages we'll request to inflate or deflate
 * the balloon in one virtqueue request. Both Linux and NetBSD
 * have settled on 256, doing up to 1MB at a time.
 */
#define VTBALLOON_PAGES_PER_REQUEST	256

/* Must be able to fix all pages frames in one page (segment). */
CTASSERT(VTBALLOON_PAGES_PER_REQUEST * sizeof(uint32_t) <= PAGE_SIZE);

#define VTBALLOON_MTX(_sc)		&(_sc)->vtballoon_mtx
#define VTBALLOON_LOCK_INIT(_sc, _name)	mtx_init(VTBALLOON_MTX((_sc)), _name, \
					    "VirtIO Balloon Lock", MTX_DEF)
#define VTBALLOON_LOCK(_sc)		mtx_lock(VTBALLOON_MTX((_sc)))
#define VTBALLOON_UNLOCK(_sc)		mtx_unlock(VTBALLOON_MTX((_sc)))
#define VTBALLOON_LOCK_DESTROY(_sc)	mtx_destroy(VTBALLOON_MTX((_sc)))

#define VTBALLOON_TEST_FLAG(_sc, _flag)	((atomic_load_32(&(_sc)->vtballoon_flags) & _flag) != 0)
#define VTBALLOON_SET_FLAG(_sc, _flag)	(atomic_set_32(&(_sc)->vtballoon_flags, _flag))

static device_method_t vtballoon_methods[] = {
	/* Device methods. */
	DEVMETHOD(device_probe,		vtballoon_probe),
	DEVMETHOD(device_attach,	vtballoon_attach),
	DEVMETHOD(device_detach,	vtballoon_detach),

	/* VirtIO methods. */
	DEVMETHOD(virtio_config_change, vtballoon_config_change),

	DEVMETHOD_END
};

static driver_t vtballoon_driver = {
	"vtballoon",
	vtballoon_methods,
	sizeof(struct vtballoon_softc)
};

VIRTIO_DRIVER_MODULE(virtio_balloon, vtballoon_driver, 0, 0);
MODULE_VERSION(virtio_balloon, 1);
MODULE_DEPEND(virtio_balloon, virtio, 1, 1, 1);

VIRTIO_SIMPLE_PNPINFO(virtio_balloon, VIRTIO_ID_BALLOON,
    "VirtIO Balloon Adapter (Traditional)");

static int
vtballoon_probe(device_t dev)
{
	return (VIRTIO_SIMPLE_PROBE(dev, virtio_balloon));
}

static int
vtballoon_attach(device_t dev)
{
	struct vtballoon_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->vtballoon_dev = dev;
	virtio_set_feature_desc(dev, vtballoon_feature_desc);

	VTBALLOON_LOCK_INIT(sc, device_get_nameunit(dev));
	TAILQ_INIT(&sc->vtballoon_pages);
	TAILQ_INIT(&sc->vtballoon_pages_deflating);

	vtballoon_setup_sysctl(sc);

	error = vtballoon_setup_features(sc);
	if (error) {
		device_printf(dev, "cannot setup features\n");
		goto fail;
	}


	error = vtballoon_alloc_virtqueues(sc);
	if (error) {
		device_printf(dev, "cannot allocate virtqueues\n");
		goto fail;
	}

	error = virtio_setup_intr(dev, INTR_TYPE_MISC);
	if (error) {
		device_printf(dev, "cannot setup virtqueue interrupts\n");
		goto fail;
	}

	sc->vtballoon_tq = taskqueue_create("vtballoon_taskq", M_NOWAIT, taskqueue_thread_enqueue, &sc->vtballoon_tq);
	vtballoon_start_taskqueues(sc);

	virtqueue_enable_intr(sc->vtballoon_inflate_vq);
	virtqueue_enable_intr(sc->vtballoon_deflate_vq);
	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_STATS_VQ) != 0) {
		virtqueue_enable_intr(sc->vtballoon_stats_vq);

		VTBALLOON_LOCK(sc);

		// initial buffer
		(void)vtballoon_send_stats(sc);

		VTBALLOON_UNLOCK(sc);
	}

	VTBALLOON_LOCK(sc);

	sc->vtballoon_desired_npages = vtballoon_desired_size(sc);
	sc->vtballoon_current_npages = 0;

	vtballoon_adjust_size(sc);

	VTBALLOON_UNLOCK(sc);

	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_PAGE_POISON) != 0) {
		sc->vtballoon_poison_val[0] = 0xde;
		sc->vtballoon_poison_val[1] = 0xad;
		sc->vtballoon_poison_val[2] = 0xbe;
		sc->vtballoon_poison_val[3] = 0xef;

		virtio_write_dev_config_4(sc->vtballoon_dev,
		    offsetof(struct virtio_balloon_config, poison_val), *((uint32_t *)sc->vtballoon_poison_val));
	}

fail:
	if (error)
		vtballoon_detach(dev);

	return (error);
}

static int
vtballoon_detach(device_t dev)
{
	struct vtballoon_softc *sc;
	struct vtballoon_page_frames *c;
	int last;

	sc = device_get_softc(dev);

	VTBALLOON_SET_FLAG(sc, VTBALLOON_FLAG_DETACH);

	if (device_is_attached(dev)) {
		vtballoon_pop(sc);

		vtballoon_drain_taskqueues(sc);

		last = 0;
		while ((c = virtqueue_drain(sc->vtballoon_inflate_vq, &last)) != NULL) {
			if (c->buf != NULL)
				free(c->buf, M_DEVBUF);
			free(c, M_DEVBUF);
		}

		last = 0;
		while ((c = virtqueue_drain(sc->vtballoon_deflate_vq, &last)) != NULL) {
			if (c->buf != NULL)
				free(c->buf, M_DEVBUF);
			free(c, M_DEVBUF);
		}

		last = 0;
		while ((c = virtqueue_drain(sc->vtballoon_stats_vq, &last)) != NULL);

		vtballoon_stop(sc);
	}

	vtballoon_free_taskqueues(sc);

	VTBALLOON_LOCK_DESTROY(sc);

	return (0);
}

static uint32_t
vtballoon_desired_size(struct vtballoon_softc *sc)
{
	uint32_t desired;

	desired = virtio_read_dev_config_4(sc->vtballoon_dev,
	    offsetof(struct virtio_balloon_config, num_pages));

	return (le32toh(desired));
}

static void
vtballoon_write_current_npages(struct vtballoon_softc *sc)
{
	uint32_t npages;

	npages = sc->vtballoon_current_npages;

	virtio_write_dev_config_4(sc->vtballoon_dev,
	    offsetof(struct virtio_balloon_config, actual), htole32(npages));
}

static int
vtballoon_config_change(device_t dev)
{
	struct vtballoon_softc *sc;

	sc = device_get_softc(dev);

	taskqueue_enqueue(taskqueue_thread, &sc->vtballoon_configchanged_task);

	return (1);
}

static void
vtballoon_start_taskqueues(struct vtballoon_softc *sc)
{
	device_t dev;
	int error;

	dev = sc->vtballoon_dev;

	TASK_INIT(&sc->vtballoon_configchanged_task, 0, vtballoon_configchanged_task, sc);
	TASK_INIT(&sc->vtballoon_deflate_response_task, 0, vtballoon_deflate_response_task, sc);
	TASK_INIT(&sc->vtballoon_inflate_response_task, 0, vtballoon_inflate_response_task, sc);

	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_STATS_VQ) != 0) {
		TASK_INIT(&sc->vtballoon_stats_request_task, 0, vtballoon_stats_request_task, sc);
	}

	error = taskqueue_start_threads(&sc->vtballoon_tq, 1, 0, "%s vtballoon taskq", device_get_nameunit(dev));
	if (error) {
		device_printf(sc->vtballoon_dev, "failed to start %s vtballoon taskq\n", device_get_nameunit(dev));
	}
}

static void
vtballoon_free_taskqueues(struct vtballoon_softc *sc)
{
	taskqueue_free(sc->vtballoon_tq);
}

static void
vtballoon_drain_taskqueues(struct vtballoon_softc *sc)
{
	taskqueue_drain(sc->vtballoon_tq, &sc->vtballoon_configchanged_task);

	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_STATS_VQ) != 0) {
		taskqueue_drain(sc->vtballoon_tq, &sc->vtballoon_stats_request_task);
	}
}

static void
vtballoon_adjust_size(struct vtballoon_softc *sc) {
	uint32_t current, desired;

	current = sc->vtballoon_current_npages;
	desired = sc->vtballoon_desired_npages;

	if (desired != current) {
		if (desired > current) {
			vtballoon_inflate(sc, (desired - current));
		} else {
			vtballoon_deflate(sc, (current - desired));
		}
	}
}

static void
vtballoon_configchanged_task(void *xsc, int pending __unused)
{
	struct vtballoon_softc *sc;
	uint32_t desired;

	sc = xsc;

	if (VTBALLOON_TEST_FLAG(sc, VTBALLOON_FLAG_DETACH)) {
		return;
	}

	VTBALLOON_LOCK(sc);

	desired = vtballoon_desired_size(sc);
	sc->vtballoon_desired_npages = desired;

	device_printf(sc->vtballoon_dev, "resizing balloon to %u pages\n", desired);

	vtballoon_adjust_size(sc);

	VTBALLOON_UNLOCK(sc);
}

static void
vtballoon_stats_request_task(void *xsc, int pending __unused)
{
	struct vtballoon_softc *sc;
	//void *c;

	sc = xsc;
	
	if (VTBALLOON_TEST_FLAG(sc, VTBALLOON_FLAG_DETACH)) {
		return;
	}

	VTBALLOON_LOCK(sc);

	//if ((c = virtqueue_dequeue(sc->vtballoon_stats_vq, NULL)) == NULL) {
	//	VTBALLOON_UNLOCK(sc);
	//	return;
	//}
	//KASSERT(c == sc->vtballoon_stats_vq, ("unexpected balloon operation response"));

	(void)vtballoon_send_stats(sc);

	VTBALLOON_UNLOCK(sc);
}

static int
vtballoon_negotiate_features(struct vtballoon_softc *sc)
{
	device_t dev;
	uint64_t features;

	dev = sc->vtballoon_dev;
	features = VTBALLOON_FEATURES;

	sc->vtballoon_features = virtio_negotiate_features(dev, features);
	return (virtio_finalize_features(dev));
}

static int
vtballoon_setup_features(struct vtballoon_softc *sc)
{
	return (vtballoon_negotiate_features(sc));
}

static int
vtballoon_alloc_virtqueues(struct vtballoon_softc *sc)
{
	device_t dev;
	struct vq_alloc_info vq_info[5];
	int nvqs;
	uint64_t features;

	dev = sc->vtballoon_dev;
	features = sc->vtballoon_features;
	nvqs = 2;

	VQ_ALLOC_INFO_INIT(&vq_info[0], 0, vtballoon_vq_intr, sc,
	    &sc->vtballoon_inflate_vq, "%s inflate", device_get_nameunit(dev));

	VQ_ALLOC_INFO_INIT(&vq_info[1], 0, vtballoon_vq_intr, sc,
	    &sc->vtballoon_deflate_vq, "%s deflate", device_get_nameunit(dev));

	if ((features & VIRTIO_BALLOON_F_STATS_VQ) != 0) {
		VQ_ALLOC_INFO_INIT(&vq_info[2], 0, vtballoon_vq_intr, sc,
		    &sc->vtballoon_stats_vq, "%s stats", device_get_nameunit(dev));
		nvqs = 3;
	}

	if ((features & VIRTIO_BALLOON_F_FREE_PAGE_HINT) != 0) {
		VQ_ALLOC_INFO_INIT(&vq_info[3], 0, vtballoon_vq_intr, sc,
		    &sc->vtballoon_free_page_vq, "%s free page", device_get_nameunit(dev));
		nvqs = 4;
	}

	if ((features & VIRTIO_BALLOON_F_PAGE_REPORTING) != 0) {
		VQ_ALLOC_INFO_INIT(&vq_info[4], 0, vtballoon_vq_intr, sc,
		    &sc->vtballoon_reporting_vq, "%s reporting", device_get_nameunit(dev));
		nvqs = 5;
	}

	return (virtio_alloc_virtqueues(dev, nvqs, vq_info));
}

static void
vtballoon_vq_intr(void *xsc)
{
	struct vtballoon_softc *sc;
	uint64_t features;
	void *c;

	sc = xsc;
	features = atomic_load_64(&sc->vtballoon_features);

	VTBALLOON_LOCK(sc);
device_printf(sc->vtballoon_dev, "intr\n");

	if (!virtqueue_empty(sc->vtballoon_inflate_vq)) {
device_printf(sc->vtballoon_dev, "..inflate\n");
		taskqueue_enqueue(taskqueue_thread, &sc->vtballoon_inflate_response_task);
	}

	if (!virtqueue_empty(sc->vtballoon_deflate_vq)) {
device_printf(sc->vtballoon_dev, "..deflate\n");
		taskqueue_enqueue(taskqueue_thread, &sc->vtballoon_deflate_response_task);
	}

	if ((features & VIRTIO_BALLOON_F_STATS_VQ) != 0 && !virtqueue_empty(sc->vtballoon_stats_vq)) {
device_printf(sc->vtballoon_dev, "..stats\n");

		if ((c = virtqueue_dequeue(sc->vtballoon_stats_vq, NULL)) != NULL) {
			KASSERT(c == sc->vtballoon_stats_vq, ("unexpected balloon operation response"));
			taskqueue_enqueue(taskqueue_thread, &sc->vtballoon_stats_request_task);
		}
	}

	if ((features & VIRTIO_BALLOON_F_FREE_PAGE_HINT) != 0 && !virtqueue_empty(sc->vtballoon_free_page_vq)) {
		//taskqueue_enqueue(taskqueue_thread, &sc->vtballoon_free_page_task);
	}

	if ((features & VIRTIO_BALLOON_F_PAGE_REPORTING) != 0 && !virtqueue_empty(sc->vtballoon_reporting_vq)) {
		//taskqueue_enqueue(taskqueue_thread, &sc->vtballoon_reporting_task);
	}

	VTBALLOON_UNLOCK(sc);
}

static size_t
vtballoon_update_stats(struct vtballoon_softc *sc)
{
	int is_modern;
	size_t i, buffers_sz;
	uint64_t usedmem, buffers;


	i = 0;
	is_modern = vtballoon_modern(sc);

#define vtballoon_put_stat(_tag, _val) \
	sc->vtballoon_stats[i++] = (struct virtio_balloon_stat){.tag = virtio_gtoh16(is_modern, _tag), .val = virtio_gtoh64(is_modern, _val)}

	/* The amount of memory that has been swapped in (in bytes) */
	vtballoon_put_stat(VIRTIO_BALLOON_S_SWAP_IN, ptoa(VM_CNT_FETCH(v_swappgsin)));

	/* The amount of memory that has been swapped out to disk (in bytes) */
	vtballoon_put_stat(VIRTIO_BALLOON_S_SWAP_OUT, ptoa(VM_CNT_FETCH(v_swappgsout)));

	/* The number of major page faults that have occurred */
	vtballoon_put_stat(VIRTIO_BALLOON_S_MAJFLT, VM_CNT_FETCH(v_io_faults));

	/* The number of minor page faults that have occurred */
	vtballoon_put_stat(VIRTIO_BALLOON_S_MINFLT, VM_CNT_FETCH(v_vm_faults));

	/* The amount of memory not being used for any purpose (in bytes) */
	vtballoon_put_stat(VIRTIO_BALLOON_S_MEMFREE, ptoa((uintmax_t)vm_free_count()));

	/* The total amount of memory available (in bytes) */
	vtballoon_put_stat(VIRTIO_BALLOON_S_MEMTOT, ptoa(physmem));

	/* An estimate of how much memory is available (in bytes) for starting
	 * new applications, without pushing the system to swap */
	usedmem = (vm_active_count() + vm_inactive_count() + vm_laundry_count());
	vtballoon_put_stat(VIRTIO_BALLOON_S_AVAIL, ptoa(physmem - usedmem));

	/* The amount of memory, in bytes, that can be quickly reclaimed without
	 * additional I/O. * Typically these pages are used for caching files
	 * from disk */
	buffers_sz = sizeof(buffers);
	if (kernel_sysctlbyname(curthread, "vfs.bufspace", &buffers, &buffers_sz, NULL, 0, 0, 0) != 0)
		buffers = 0;

	vtballoon_put_stat(VIRTIO_BALLOON_S_CACHES, buffers);

	/* The number of successful hugetlb page allocations in the guest */
	/*vtballoon_put_stat(VIRTIO_BALLOON_S_HTLB_PGALLOC, 0);*/

	/* The number of failed hugetlb page allocations in the guest */
	/*vtballoon_put_stat(VIRTIO_BALLOON_S_HTLB_PGFAIL, 0);*/

	KASSERT(i < VIRTIO_BALLOON_S_NR, ("array overflow"));

#undef vtballoon_put_stat

	return (i);
}

static int
vtballoon_send_stats(struct vtballoon_softc *sc)
{
	int error __diagused;
	size_t num_stats;
	struct sglist sg;
	struct sglist_seg segs[1];

	num_stats = vtballoon_update_stats(sc);

	sglist_init(&sg, 1, segs);
	error = sglist_append(&sg, sc->vtballoon_stats, sizeof(sc->vtballoon_stats[0])*num_stats);
	KASSERT(error == 0, ("error outputting stats buffer to virtqueue"));
	if (error != 0) return (error);

	error = virtqueue_enqueue(sc->vtballoon_stats_vq, sc->vtballoon_stats_vq, &sg, 1, 0);
	KASSERT(error == 0, ("error enqueuing memory stats to virtqueue"));
	if (error != 0) return (error);

	virtqueue_notify(sc->vtballoon_stats_vq);

	return (0);
}


static int
vtballoon_send_page_frames(struct vtballoon_softc *sc, struct virtqueue *vq,
    struct vtballoon_page_frames *page_frames)
{
	struct sglist sg;
	struct sglist_seg segs[1];
	uint32_t *virtio_page_frames;
	int i;
	int error __diagused;

	virtio_page_frames = NULL;
	error = 0;

	sglist_init(&sg, 1, segs);

	virtio_page_frames = malloc((sizeof(*virtio_page_frames) * page_frames->npages), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (virtio_page_frames == NULL) {
		error = 1;
		device_printf(sc->vtballoon_dev,
		    "cannot allocate page frame request array (ENOMEM)\n");
		goto err;
	}

	for (i = 0; i < page_frames->npages; i++) {
		virtio_page_frames[0] = VM_PAGE_TO_PHYS(page_frames->pages[i]) >> VIRTIO_BALLOON_PFN_SHIFT;
	}

	page_frames->buf = virtio_page_frames;

	error = sglist_append(&sg, virtio_page_frames, (page_frames->npages * sizeof(*virtio_page_frames)));
	//KASSERT(error == 0, ("error adding page frames to sglist"));
	if (error != 0) { goto err; }

	error = virtqueue_enqueue(vq, page_frames, &sg, 1, 0);
	//KASSERT(error == 0, ("error enqueuing page frames to virtqueue"));
	if (error != 0) { goto err; }

	virtqueue_notify(vq);

	return (0);

err:
	if (virtio_page_frames != NULL) {
		free(virtio_page_frames, M_DEVBUF);
	}
	return (error);
}

static void
vtballoon_inflate(struct vtballoon_softc *sc, int npages)
{
	struct virtqueue *vq;
	struct vtballoon_page_frames *page_frames;
	vm_page_t m;
	int i;

	vq = sc->vtballoon_inflate_vq;

	if (VTBALLOON_TEST_FLAG(sc, VTBALLOON_FLAG_DETACH)) {
		// don't inflate the balloon when we're already unloading the kmod
		return;
	}

	if (npages > VTBALLOON_PAGES_PER_REQUEST) {
		npages = VTBALLOON_PAGES_PER_REQUEST;
	}

	page_frames = malloc((sizeof(struct vtballoon_page_frames) + npages * sizeof(*page_frames->pages)), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (page_frames == NULL) {
		device_printf(sc->vtballoon_dev,
		    "cannot allocate page frame array (ENOMEM)\n");
		return;
	}

	for (i = 0; i < npages; i++) {
		if ((m = vtballoon_alloc_page(sc)) == NULL) {
			sc->vtballoon_timeout = VTBALLOON_LOWMEM_TIMEOUT;
			break;
		}

		page_frames->pages[i] = m;

		KASSERT(m->a.queue == PQ_NONE,
		    ("%s: allocated page %p on queue", __func__, m));
		TAILQ_INSERT_TAIL(&sc->vtballoon_pages, m, plinks.q);
	}
	page_frames->npages = i;

	if (page_frames->npages > 0) {
		vtballoon_send_page_frames(sc, vq, page_frames);
	} else {
		free(page_frames, M_DEVBUF);
	}
}

static void
vtballoon_inflate_response_task(void *xsc, int pending __unused)
{
	struct vtballoon_softc *sc;
	struct vtballoon_page_frames *c;

	sc = xsc;

	VTBALLOON_LOCK(sc);

	c = virtqueue_dequeue(sc->vtballoon_inflate_vq, NULL);
device_printf(sc->vtballoon_dev, "inflate cookie: %p\n", c);
	if (c != NULL) {
		if (c->buf != NULL) {
			free(c->buf, M_DEVBUF);
		}
		free(c, M_DEVBUF);

		vtballoon_write_current_npages(sc);

		if (!VTBALLOON_TEST_FLAG(sc, VTBALLOON_FLAG_DETACH)) {
			vtballoon_adjust_size(sc);
		}
	}

	VTBALLOON_UNLOCK(sc);
}

static void
vtballoon_deflate(struct vtballoon_softc *sc, int npages)
{
	struct virtqueue *vq;
	struct vtballoon_page_frames *page_frames;
	vm_page_t m;
	int i;

	vq = sc->vtballoon_deflate_vq;

	if (npages > VTBALLOON_PAGES_PER_REQUEST) {
		npages = VTBALLOON_PAGES_PER_REQUEST;
	}
	if (npages > sc->vtballoon_current_npages) {
		npages = sc->vtballoon_current_npages;
	}

	page_frames = malloc((sizeof(struct vtballoon_page_frames) + npages * sizeof(*page_frames->pages)), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (page_frames == NULL) {
		device_printf(sc->vtballoon_dev,
		    "cannot allocate page frame array (ENOMEM)\n");
		return;
	}

	for (i = 0; i < npages; i++) {
		m = TAILQ_FIRST(&sc->vtballoon_pages);
		KASSERT(m != NULL, ("%s: no more pages to deflate", __func__));

		page_frames->pages[i] = m;

		TAILQ_REMOVE(&sc->vtballoon_pages, m, plinks.q);
		TAILQ_INSERT_TAIL(&sc->vtballoon_pages_deflating, m, plinks.q);
	}
	page_frames->npages = npages = i;

	if (npages < 1) { goto revert; }

	/* Always tell host first before freeing the pages. */
	if (vtballoon_send_page_frames(sc, vq, page_frames) != 0) {
		goto revert;
	}

	return;

revert:
	for (i = 0; i < npages; i++) {
		m = page_frames->pages[i];

		TAILQ_REMOVE(&sc->vtballoon_pages_deflating, m, plinks.q);
		TAILQ_INSERT_TAIL(&sc->vtballoon_pages, m, plinks.q);
	}

	free(page_frames, M_DEVBUF);
}

static void
vtballoon_deflate_response_task(void *xsc, int pending __unused)
{
	struct vtballoon_softc *sc;
	int i;
	struct vtballoon_page_frames *c;

	sc = xsc;

	VTBALLOON_LOCK(sc);

	if ((c = virtqueue_dequeue(sc->vtballoon_deflate_vq, NULL)) == NULL) {
		goto ret;
	}

	for (i = 0; i < c->npages; i++) {
		vm_page_t m = c->pages[i];

		TAILQ_REMOVE(&sc->vtballoon_pages_deflating, m, plinks.q);
		vtballoon_free_page(sc, m);
	}

	if (c->buf != NULL) {
		free(c->buf, M_DEVBUF);
	}
	free(c, M_DEVBUF);

	KASSERT((TAILQ_EMPTY(&sc->vtballoon_pages) &&
	    sc->vtballoon_current_npages == 0) ||
	    (!TAILQ_EMPTY(&sc->vtballoon_pages) &&
	    sc->vtballoon_current_npages != 0),
	    ("%s: bogus page count %d", __func__,
	    sc->vtballoon_current_npages));

	vtballoon_write_current_npages(sc);

	wakeup(&sc->vtballoon_pages);

	vtballoon_adjust_size(sc);

ret:
	VTBALLOON_UNLOCK(sc);
}

static void
vtballoon_pop(struct vtballoon_softc *sc)
{
	VTBALLOON_LOCK(sc);

	// wait until the balloon has popped and the device has acknowledged all pages
	while (!TAILQ_EMPTY(&sc->vtballoon_pages) || !TAILQ_EMPTY(&sc->vtballoon_pages_deflating)) {
		vtballoon_deflate(sc, sc->vtballoon_current_npages);

		msleep(&sc->vtballoon_pages, VTBALLOON_MTX(sc), 0, "vtbpop", 50);
	}

	VTBALLOON_UNLOCK(sc);
}

static void
vtballoon_stop(struct vtballoon_softc *sc)
{
	virtqueue_disable_intr(sc->vtballoon_inflate_vq);
	virtqueue_disable_intr(sc->vtballoon_deflate_vq);
	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_STATS_VQ) != 0) {
		virtqueue_disable_intr(sc->vtballoon_stats_vq);
	}
	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_FREE_PAGE_HINT) != 0) {
		virtqueue_disable_intr(sc->vtballoon_free_page_vq);
	}
	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_PAGE_REPORTING) != 0) {
		virtqueue_disable_intr(sc->vtballoon_reporting_vq);
	}

	virtio_stop(sc->vtballoon_dev);
}

static vm_page_t
vtballoon_alloc_page(struct vtballoon_softc *sc)
{
	vm_page_t m;

	m = vm_page_alloc_noobj(VM_ALLOC_NODUMP);
	if (m != NULL) {
		sc->vtballoon_current_npages++;
	}

	return (m);
}

static void
vtballoon_poison_page(struct vtballoon_softc *sc, vm_page_t m) {
/*	TODO
	uint64_t i, s;
	uint32_t *p;
	uint8_t *v;

	p = (uint32_t *)m->phys_addr;
	v = sc->vtballoon_poison_val;
	s = VM_CNT_FETCH(v_page_size);

	for (i = 0; i < s; i += 4) {
		p[i] = p;
	} */
}

static void
vtballoon_free_page(struct vtballoon_softc *sc, vm_page_t m)
{
	if ((sc->vtballoon_features & VIRTIO_BALLOON_F_PAGE_POISON) != 0) {
		vtballoon_poison_page(sc, m);
	}

	vm_page_free(m);
	sc->vtballoon_current_npages--;
}

static void
vtballoon_setup_sysctl(struct vtballoon_softc *sc)
{
	device_t dev;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	struct sysctl_oid_list *child;

	dev = sc->vtballoon_dev;
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);
	child = SYSCTL_CHILDREN(tree);

	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "desired",
	    CTLFLAG_RD, &sc->vtballoon_desired_npages, sizeof(uint32_t),
	    "Desired balloon size in pages");

	SYSCTL_ADD_UINT(ctx, child, OID_AUTO, "current",
	    CTLFLAG_RD, &sc->vtballoon_current_npages, sizeof(uint32_t),
	    "Current balloon size in pages");
}
