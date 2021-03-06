/*
 *
 * device driver for TW6868 based PCIe capture cards
 * driver core for hardware
 *
 */

#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kmod.h>
#include <linux/sound.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/pm.h>
#include <linux/vmalloc.h>  // liutest add

#include "TW68.h"
#include "TW68_defines.h"


MODULE_DESCRIPTION("v4l2 driver module for TW6868/6869 based CVBS video capture cards");
MODULE_AUTHOR("Simon Xu 2011 @intersil");
MODULE_LICENSE("GPL");

/// 0819 vma buffer   modprobe videobuf_vmalloc
/* ------------------------------------------------------------------ */

static unsigned int irq_debug;
module_param(irq_debug, int, 0644);
MODULE_PARM_DESC(irq_debug,"enable debug messages [IRQ handler]");

static unsigned int core_debug;
module_param(core_debug, int, 0644);
MODULE_PARM_DESC(core_debug,"enable debug messages [core]");

static unsigned int gpio_tracking;
module_param(gpio_tracking, int, 0644);
MODULE_PARM_DESC(gpio_tracking,"enable debug messages [gpio]");

static unsigned int alsa = 1;
module_param(alsa, int, 0644);
MODULE_PARM_DESC(alsa,"enable/disable ALSA DMA sound [dmasound]");

static unsigned int latency = UNSET;
module_param(latency, int, 0444);
MODULE_PARM_DESC(latency,"pci latency timer");

int TW68_no_overlay=-1;
///module_param_named(no_overlay, TW6869_no_overlay, int, 0444);
///MODULE_PARM_DESC(no_overlay,"allow override overlay default (0 disables, 1 enables)"
///		" [some VIA/SIS chipsets are known to have problem with overlay]");

static unsigned int video_nr[] = {[0 ... (TW68_MAXBOARDS - 1)] = UNSET };
static unsigned int vbi_nr[]   = {[0 ... (TW68_MAXBOARDS - 1)] = UNSET };
static unsigned int radio_nr[] = {[0 ... (TW68_MAXBOARDS - 1)] = UNSET };
static unsigned int tuner[]    = {[0 ... (TW68_MAXBOARDS - 1)] = UNSET };
static unsigned int card[]     = {[0 ... (TW68_MAXBOARDS - 1)] = UNSET };


module_param_array(video_nr, int, NULL, 0444);
module_param_array(vbi_nr,   int, NULL, 0444);
module_param_array(radio_nr, int, NULL, 0444);
module_param_array(tuner,    int, NULL, 0444);
module_param_array(card,     int, NULL, 0444);

MODULE_PARM_DESC(video_nr, "video device number");
MODULE_PARM_DESC(vbi_nr,   "vbi device number");
MODULE_PARM_DESC(radio_nr, "radio device number");
MODULE_PARM_DESC(tuner,    "tuner type");
MODULE_PARM_DESC(card,     "card type");

DEFINE_MUTEX(TW686v_devlist_lock);
EXPORT_SYMBOL(TW686v_devlist_lock);
LIST_HEAD(TW686v_devlist);
EXPORT_SYMBOL(TW686v_devlist);

#define container_of(ptr, type, member) \
	({  const typeof( ((type *)0)->member ) *__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member) );})


/* ------------------------------------------------------------------ */

//static char name_mute[]    = "mute";
//static char name_radio[]   = "Radio";
//static char name_tv[]      = "Television";
//static char name_tv_mono[] = "TV (mono only)";
//static char name_comp[]    = "Composite";
static char name_comp1[]   = "Composite1";
static char name_comp2[]   = "Composite2";
static char name_comp3[]   = "Composite3";
static char name_comp4[]   = "Composite4";
//static char name_svideo[]  = "S-Video";

/* ------------------------------------------------------------------ */
/* board config info                                                  */

static DEFINE_MUTEX(TW68_devlist_lock);

struct TW68_board TW68_boards[] = {
	[TW68_BOARD_UNKNOWN] = {
		.name		= "TW6869",
		.audio_clock	= 0,
		.tuner_type	= TUNER_ABSENT,
		.radio_type     = UNSET,
		.tuner_addr	= ADDR_UNSET,
		.radio_addr	= ADDR_UNSET,

		.inputs         = {{
			.name = name_comp1,
			.vmux = 0,
			.amux = LINE1,
		}},
	},

    [TW68_BOARD_A] = {
		.name		= "TW6869",
		.audio_clock	= 0,
		.tuner_type	= TUNER_ABSENT,
		.radio_type     = UNSET,
		.tuner_addr	= ADDR_UNSET,
		.radio_addr	= ADDR_UNSET,

		.inputs         = {{
			.name = name_comp1,
			.vmux = 0,
			.amux = LINE1,
		},{
			.name = name_comp2,
			.vmux = 1,
			.amux = LINE2,
		},{
			.name = name_comp3,
			.vmux = 2,
			.amux = LINE3,
		},{
			.name = name_comp4,
			.vmux = 4,
			.amux = LINE4,
		}},
	},

};

const unsigned int TW68_bcount = ARRAY_SIZE(TW68_boards);

/* ------------------------------------------------------------------ */
/* PCI ids + subsystem IDs                                            */

struct pci_device_id TW68_pci_tbl[] = {

	{
		.vendor       = 0x1797,
		.device       = 0x6869,
		.subvendor    = 0,
		.subdevice    = 0,
		.driver_data  = 0,
	},

	/*
	{
		.vendor       = 0x1797,
		.device       = 0x6864,
		.subvendor    = 0,
		.subdevice    = 0,
		.driver_data  = 0,
	},
	*/
	//
	{
		/* --- end of list --- */
	}
};
/* ------------------------------------------------------------------ */
MODULE_DEVICE_TABLE(pci, TW68_pci_tbl);

static LIST_HEAD(mops_list);
static unsigned int TW68_devcount;


static u32 video_framerate[3][6] = {
	{
			0xBFFFFFFF,	  /// 30  FULL
			0xBFFFCFFF,  /// 28
			0x8FFFCFFF,  /// 26
		    0xBF3F3F3F,  /// 24 FPS
			0xB3CFCFC3,  /// 22
            0x8F3CF3CF  /// 20 FPS
	},
	{		30, 28, 26, 24, 22, 20},
	{		25, 23, 20, 18, 16, 14}

};


int (*TW68_dmasound_init)(struct TW68_dev *dev);
int (*TW68_dmasound_exit)(struct TW68_dev *dev);

#define dprintk(fmt, arg...)	if (core_debug) \
	printk(KERN_DEBUG "%s/core: " fmt, dev->name , ## arg)


void tw68v_set_framerate(struct TW68_dev *dev, u32 ch, u32 n)
{
	if (n >= 0  &&  n < 6)
	{
		if(ch >=0 && ch<8)
			reg_writel( DROP_FIELD_REG0+ ch,  video_framerate[n][0]);   // 30 FPS
		printk("%%%%    tw68v_set_framerate: ch Id %d   n:%d  %d FPS \n ", ch, n,  video_framerate[n][1]);
	}
}



/* ------------------------------------------------------------------ */

void dma_field_init(struct dma_region *dma)
{
	dma->kvirt = NULL;
	dma->dev = NULL;
	dma->n_pages = 0;
	dma->n_dma_pages = 0;
	dma->sglist = NULL;
}

/**
 * dma_region_free - unmap and free the buffer
 */
void dma_field_free(struct dma_region *dma)
{
	if (dma->n_dma_pages) {
		pci_unmap_sg(dma->dev, dma->sglist, dma->n_pages,
			     dma->direction);
		dma->n_dma_pages = 0;
		dma->dev = NULL;
	}

	vfree(dma->sglist);
	dma->sglist = NULL;

	vfree(dma->kvirt);
	dma->kvirt = NULL;
	dma->n_pages = 0;
}

/**   struct pci_dev *pci,
 * dma_region_alloc - allocate the buffer and map it to the IOMMU
 */
int dma_field_alloc(struct dma_region *dma, unsigned long n_bytes,
		     struct pci_dev *dev, int direction)
{
	unsigned int i;

	/* round up to page size */
	/*  to align the pointer to the (next) page boundary
		#define PAGE_ALIGN(addr)        (((addr) + PAGE_SIZE - 1) & PAGE_MASK)
		this worked as PAGE_SIZE and PAGE_MASK were available in page.h.
	*/

	n_bytes = PAGE_ALIGN(n_bytes);

	dma->n_pages = n_bytes >> PAGE_SHIFT;
	/// printk( "dma_field_alloc: n_bytes %d   n_pages %d\n", n_bytes, dma->n_pages  );


	dma->kvirt = vmalloc_32(n_bytes);
	if (!dma->kvirt) {
		/// printk(KERN_ERR "dma_region_alloc: vmalloc_32() failed\n");
		goto err;
	}

	/* Clear the ram out, no junk to the user */
	memset(dma->kvirt, 0, n_bytes);

	/* allocate scatter/gather list */
	dma->sglist = vmalloc(dma->n_pages * sizeof(*dma->sglist));
	if (!dma->sglist) {
		/// printk(KERN_ERR "dma_region_alloc: vmalloc(sglist) failed\n");
		goto err;
	}

	sg_init_table(dma->sglist, dma->n_pages);

	/* fill scatter/gather list with pages */
	for (i = 0; i < dma->n_pages; i++) {
		unsigned long va =
		    (unsigned long)dma->kvirt + (i << PAGE_SHIFT);

		sg_set_page(&dma->sglist[i], vmalloc_to_page((void *)va),
				PAGE_SIZE, 0);
	}

	/* map sglist to the IOMMU */
	dma->n_dma_pages =
	    pci_map_sg(dev, dma->sglist, dma->n_pages, direction);

	if (dma->n_dma_pages == 0) {
		/// printk(KERN_ERR "dma_region_alloc: pci_map_sg() failed\n");
		goto err;
	}

	dma->dev = dev;
	dma->direction = direction;

	return 0;

  err:
	dma_field_free(dma);
	return -ENOMEM;
}



int TW68_buffer_pages(int size)
{
	size  = PAGE_ALIGN(size);
	size += PAGE_SIZE; /* for non-page-aligned buffers */
	size /= 4096;
	return size;
}

/* calc max # of buffers from size (must not exceed the 4MB virtual
 * address space per DMA channel) */
int TW68_buffer_count(unsigned int size, unsigned int count)
{
	unsigned int maxcount;

	maxcount = 1024 / TW68_buffer_pages(size);
	if (count > maxcount)
		count = maxcount;


	 /// printk("TW68_buffer_count  size %d   maxcount %d / C %d \n",  size,	maxcount, count );

	return count;
}

int TW68_buffer_startpage(struct TW68_buf *buf)
{
    unsigned long pages, n, pgn;
    pages = TW68_buffer_pages(buf->vb.bsize);
    n = buf->vb.i;
    pgn = pages * n;
    /// printk("TW68_buffer_startpage  = %d / %p %d %d =%d\n",
    ///        pages, (buf->vb.bsize), (buf->vb.bsize), n, pgn);
	return pgn;
}

unsigned long TW68_buffer_base(struct TW68_buf *buf)
{
	unsigned long base0, base;
	struct videobuf_dmabuf *dma=videobuf_to_dma(&buf->vb);

    // void* vbuf = videobuf_to_vmalloc(&buf->vb);

	base0  = TW68_buffer_startpage(buf) * 4096;
	base = base0 + dma->sglist[0].offset;

	/// printk("BBBBBB dma%x  vma%x  vbuf i%x  TW68_buffer_base  base0=%x / base =%x   offset %x\n",
    ///        dma->bus_addr,  dma->vmalloc, buf->vb.baddr,  base0, base, dma->sglist[0].offset);

	return base;
}

/* ------------------------------------------------------------------ */


int AudioDMA_PB_alloc(struct pci_dev *pci, struct TW68_pgtable *pt)
{
	dma_addr_t   dma_addr;
    	int audio_ch;
	//__le32 *clean; 
	__le32 *pdmaP, *pdmaB;
	__le32 *AudioPages;

	// for audio  CH  P/B

	AudioPages = pci_alloc_consistent(pci, PAGE_SIZE * MAX_NUM_DATA_DMA * 2, &dma_addr);

	if (NULL == AudioPages)
	{
		return -ENOMEM;
	}

	pt->size = PAGE_SIZE * MAX_NUM_DATA_DMA * 2;  ///pt;  //2
	pt->cpu  = AudioPages;
	pt->dma  = dma_addr;

	for (audio_ch =0; audio_ch < MAX_NUM_DATA_DMA; audio_ch++)
	{
		pdmaP = pt->cpu + (PAGE_SIZE <<1)* audio_ch /4 + 100;
		pdmaB = pt->cpu + ((PAGE_SIZE <<1)* audio_ch + PAGE_SIZE)/4 + 100;

	}
	return 0;
}


int videoDMA_pgtable_alloc(struct pci_dev *pci, struct TW68_pgtable *pt)
{
	__le32      *cpu;
	// __le32		*clean;
	dma_addr_t   dma_addr, phy_addr;

	cpu = pci_alloc_consistent(pci, PAGE_SIZE<<3, &dma_addr);   // 8* 4096 contiguous  //*2

	if (NULL == cpu)
	{
		return -ENOMEM;
	}

	pt->size = PAGE_SIZE<<3;  ///pt;  //2
	pt->cpu  = cpu;
	pt->dma  = dma_addr;
	phy_addr = dma_addr + (PAGE_SIZE<<2) + (PAGE_SIZE<<1);  //6 pages

    ///printk("  videoDMA_pgtable_alloc: cpu:0X%p  pt->size: 0x%x   BD:0X%x  \n", cpu, pt->size, (unsigned int)pt->cpu + pt->size );

	/*
    for ( clean = cpu; (unsigned int)clean < ((unsigned int)pt->cpu + pt->size); clean++ )
    {
			*clean++ = 0x40001000;  // ctrl dw
			//  *clean = phy_addr;
			if ((((unsigned int)pt->cpu + pt->size)-(unsigned int)clean) <6)
           printk("---TW68_pgtable_alloc--- mem%0d   write CPU 0X%p  SIZE%d, clean 0X%p = 0X%x 0X%x   \n",
           ((unsigned int)(clean-1) - (unsigned int)cpu), cpu, pt->size, clean,  *(clean-1), *clean );
    }
	*/
	return 0;
}





void TW68_pgtable_free(struct pci_dev *pci, struct TW68_pgtable *pt)
{
	if (NULL == pt->cpu)
		return;
	pci_free_consistent(pci, pt->size, pt->cpu, pt->dma);
	pt->cpu = NULL;
}

/* ------------------------------------------------------------------ */
/*
void TW68_dma_free(struct videobuf_queue *q,struct TW68_buf *buf)
{
	struct videobuf_dmabuf *dma=videobuf_to_dma(&buf->vb);
	BUG_ON(in_interrupt());

    /// printk("  $$$$ _dma_free :: dma->vmalloc 0x%X  vb->baddr %p \n", dma->vmalloc, buf->vb.baddr);

// 6.0	
	//videobuf_waiton(&buf->vb,0,0);
// 6.1
	videobuf_waiton( q, &buf->vb,0,0);

	//videobuf_dma_unmap(q, dma);

	videobuf_dma_free(dma);
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}
*/
/* ------------------------------------------------------------------ */

int TW68_buffer_queue(struct TW68_dev *dev,
			 struct TW68_dmaqueue *q,
			 struct TW68_buf *buf)
{
	///printk(" %s: video buffer_queue %p    buf %p \n", __func__, q, buf);

	/*
	if (!list_empty(&q->queued)) {
		list_add_tail(&buf->vb.queue, &q->queued);
		buf->vb.state = VIDEOBUF_QUEUED;
		printk( "%s: [%p/%d] appended to queued\n",
			__func__, buf, buf->vb.i);

	// else if the 'active' chain doesn't yet exist we create it now 
	} 
	else 
		if (list_empty(&q->active)) {
		printk( "%s: [%p/%d] first active\n",
			__func__, buf, buf->vb.i);
		list_add_tail(&buf->vb.queue, &q->active);
		//  TODO - why have we removed buf->count and q->count? 
		buf->activate(dev, buf, NULL);

		if (NULL == q->curr) {
			q->curr = buf;
		}
	}
	*/
	/* else we would like to put this buffer on the tail of the
	 * active chain, provided it is "compatible". */

	
	if (NULL == q->curr) {
			q->curr = buf;
			buf->activate(dev,buf,NULL);
	} else
	{
		list_add_tail(&buf->vb.queue,&q->queued);     // curr
		buf->vb.state = VIDEOBUF_QUEUED;
	}
	
	//		TW68_buffer_requeue(dev, q);  //&dev->video_dmaq[nId]);

	return 0;
	 
}


/* ------------------------------------------------------------------ */
/*
 * Buffer handling routines
 *
 * These routines are "generic", i.e. are intended to be used by more
 * than one module, e.g. the video and the transport stream modules.
 * To accomplish this generality, callbacks are used whenever some
 * module-specific test or action is required.
 */

/* resends a current buffer in queue after resume */
int TW68_buffer_requeue(struct TW68_dev *dev,
				  struct TW68_dmaqueue *q)
{
	struct TW68_buf *buf, *prev;

	printk( "%s: called\n", __func__);

	if (!list_empty(&q->active)) {
		buf = list_entry(q->active.next, struct TW68_buf, vb.queue);
		// printk( "%s: [%p/%d] restart dma\n", __func__,	buf, buf->vb.i);
		//q->start_dma(dev, q, buf);
		mod_timer(&q->timeout, jiffies + BUFFER_TIMEOUT);
		return 0;
	}

	prev = NULL;
	for (;;) {
		if (list_empty(&q->queued))
			return 0;
		buf = list_entry(q->queued.next, struct TW68_buf, vb.queue);
		/* if nothing precedes this one */
		if (NULL == prev) {
			list_move_tail(&buf->vb.queue, &q->active);
			buf->activate(dev, buf, NULL);
			printk( "%s: [%p/%d] first active\n",
				__func__, buf, buf->vb.i);

		} 
		else 
		{
			list_move_tail(&buf->vb.queue, &q->active);
			buf->activate(dev, buf, NULL);
			/// printk( "%s: [%p/ %p] move to active\n", __func__, buf, buf->vb.i);
		} 
{
			//printk( "%s: no action taken\n", __func__);
			return 0;
		}
		prev = buf;

		return 0;
		
	}
}


void TW68_buffer_finish(struct TW68_dev *dev,
			   struct TW68_dmaqueue *q,
			   unsigned int state)
{

mutex_lock(&q->lock); //  liutest
	if (q->dev != dev)	return;
	q->curr->vb.state = state;
	do_gettimeofday(&q->curr->vb.ts);
mutex_unlock(&q->lock); //  liutest
	/// printk("_buffer_finish ::buffer_finish %p -> NULL  k %p \n",q->curr, k);
	//printk("liutest 1 _buffer_finish ::buffer_finish %p -> NULL\n",q->curr);
	wake_up(&q->curr->vb.done);
	//printk("liutest 2 _buffer_finish ::buffer_finish %p -> NULL\n",q->curr);
	q->curr = NULL;

}

void TW68_buffer_next(struct TW68_dev *dev,
			 struct TW68_dmaqueue *q)
{
	struct TW68_buf *buf,*next = NULL;
	//printk("liutest 1 TW68_buffer_next ::TW68_buffer_next %p ->\n",q->curr);
	mutex_lock(&q->lock); //  liutest 
	//-BUG_ON(NULL != q->curr);
	//printk("liutest 2 TW68_buffer_next ::TW68_buffer_next %p ->\n",q->curr);

	if (!list_empty(&q->queued)) {
		/* activate next one from  dma queue */
		buf = list_entry(q->queued.next,struct TW68_buf,vb.queue);

		/// **** remove  v4l video buffer from the queue
		list_del(&buf->vb.queue);
		if (!list_empty(&q->queued))
			next = list_entry(q->queued.next, struct TW68_buf,  vb.queue);
		q->curr = buf;

		buf->vb.state = VIDEOBUF_ACTIVE;

		///printk(KERN_INFO " $$$ buffer_activate  buf->vb.state = VIDEOBUF_ACTIVE; \n");
	       mod_timer( &q->timeout, jiffies+BUFFER_TIMEOUT);  // not liutest remove
	} else {
		/* nothing to do -- just stop DMA */
		//printk( " liutest TW68_buffer_next =>TW68_dmaqueue list is empty\n");
		del_timer(&q->timeout);    // not liutest remove 
	}
	mutex_unlock(&q->lock);  //  liutest
}


void Field_SG_Mapping(struct TW68_dev *dev, int  field_PB)  //    0 1
{
	struct TW68_dmaqueue *q;
	struct TW68_buf *buf;
	struct TW68_pgtable   *pt;
	unsigned int  i, nbytes, FieldSize, remain, nIDX, pgn;
	u32 dwCtrl;
    __le32        *ptr;
	u32 nId =0;
	u32 m_CurrentFrameStartIdx = 0; //128 *nId;
	u32 m_NextFrameStartIdx = 0;  //128 *nId;
    struct videobuf_dmabuf *dma;
	struct scatterlist *list;
	q = &dev->video_q;

	if (!list_empty(&q->queued)) {
		/* get next buffer from  dma queue */
		buf = list_entry(q->queued.next,struct TW68_buf,vb.queue);
		//
		printk("@@@@ locate buffer_next %p [prev=%p/next=%p]   &buf->vb.queue= %p \n",
			buf, q->queued.prev,q->queued.next,  &buf->vb.queue);

		// fill half frame SG mapping entries
		  dma=videobuf_to_dma(&buf->vb);
		  list = dma ->sglist;
	// dma channel offset = 8192 /8 /4 /2;
	ptr = dev->m_Page0.cpu + (2* 128 * nId);  // channel start entry address

    printk("Field_SG_Mapping  size %ld  Video Frame\n", buf->vb.size);


	pt = &dev->m_Page0;
	BUG_ON(NULL == pt || NULL == pt->cpu);

 
    nIDX = 128;   ///85;
    pgn = 83;
	// dma channel offset = 8192 /8 /4 /2;
	ptr = pt->cpu + (2* nIDX * nId);  // channel start entry address   128
	///printk("--_pgtable--nId 2 -- CPU ptr %p  P start address %p  &  dma_address %p \n", pt->cpu, ptr, pt->dma );

    FieldSize = buf->vb.size /2;

    nbytes = 0;
	for (i = 0; i < dma->sglen; i++, list++)
		///for (p = 0; p * 4096 < list->length; p++, ptr++)
		{
		    // switch to B
		    if ( ((nbytes + list->length) >= FieldSize) && ((nbytes + list->length) <=  (FieldSize + list->length)) )
			{

                remain = FieldSize - (nbytes);
                if (remain >0)
                {

                    dwCtrl = 				(((DMA_STATUS_HOST_READY &0x3)<<30) |
											(((0)&1)<<29) |
											((m_CurrentFrameStartIdx & 0xFF) << 14)  |
											((m_NextFrameStartIdx    & 0xFF) << 21)  |
											//(((m_nIncomingFrameCnt[id]>12)&(MappingNum ==0)&1)<<13)	|
											//( 1<<13)	|
											(remain & 0x1FFF));	// size

					if (field_PB)
						ptr++;
					else
						*(ptr++) = cpu_to_le32(dwCtrl);

					if (field_PB)
						ptr++;
					else
	                    *(ptr++) = cpu_to_le32(sg_dma_address(list) - list->offset );  /// setup page dma address

                    printk("--_pgtable P/B switch ptr: %p  *(ptr-1)%x, *ptr%x  nbytes%d  FieldSize%d  remain%d   i%d\n", ptr, *(ptr-1), *ptr, nbytes, FieldSize, remain, i);
                }


                remain =  (nbytes + list->length) - FieldSize;
                nbytes += list->length;

		        ptr = (pt->cpu + (2* nIDX * nId)) + 0x800;  // 2 pages distance
				printk("--_pgtable P/B i%d ->B  new ptr: %p    pt->cpu %p   nbytes%d  FieldSize%d  remain%d    \n", i, ptr, pt->cpu, nbytes, FieldSize, remain);

		             dwCtrl = 		(((DMA_STATUS_HOST_READY &0x3)<<30) |
									(((1)&1)<<29) |
									((m_CurrentFrameStartIdx & 0xFF) << 14)  |
									((m_NextFrameStartIdx    & 0xFF) << 21)  |
									//(((m_nIncomingFrameCnt[id]>12)&(MappingNum ==0)&1)<<13)	|
									//( 1<<13)	|
									(remain & 0x1FFF));	// size

 					if (field_PB)
	                    *(ptr++) = cpu_to_le32(dwCtrl);
					else
						ptr++;

					if (field_PB)
					    *(ptr++) = cpu_to_le32(sg_dma_address(list) - list->offset + list->length - remain  );  /// setup page dma address
					else
						ptr++;

                    printk("--_pgtable P/B switch ptr: %p  *(ptr-1)%x, *ptr%x  nbytes%d  FieldSize%d  remain%d    i%d\n", ptr, *(ptr-1), *ptr, nbytes, FieldSize, remain, i);

		        ///printk("--_pgtable P/B switch ptr: %p  nbytes%d  FieldSize%d  remain%d \n", ptr, nbytes, FieldSize, remain);

		    }

            else
            {

			           dwCtrl = 			(((DMA_STATUS_HOST_READY &0x3)<<30) |
											(((i==0)&1)<<29) |
											((m_CurrentFrameStartIdx & 0x7F) << 14)  |
											((m_NextFrameStartIdx    & 0xFF) << 21)  |
											(list->length & 0x1FFF));	// size

			if (((!field_PB)&& ((nbytes + list->length) <= FieldSize)) || ((field_PB)&& (nbytes >= FieldSize)))
				*(ptr++) = cpu_to_le32(dwCtrl);
			else
				ptr++; /// pointing to the Address dword

			if (((!field_PB)&& ((nbytes + list->length) <= FieldSize)) || ((field_PB)&& (nbytes >= FieldSize)))
				*(ptr++) = cpu_to_le32(sg_dma_address(list) - list->offset );  /// setup page dma address
			else
				ptr++; /// pointing to the Address dword

            nbytes += list->length;

            }

		}
	} else {
		/* nothing to do -- just stop DMA */
		printk("buffer_next %p   no more buffer \n",NULL);
	}
}


void Fixed_SG_Mapping(struct TW68_dev *dev, int nDMA_channel, int Frame_size)  //    0 1
{
	// PROGRAM DMA DESCRIPTOR WITH FIXED vma dma_region sglist
	struct dma_region			*Field_P;
	struct dma_region			*Field_B;
	struct scatterlist			*sglist;
	struct TW68_pgtable   *pt;
	int  i, nbytes, FieldSize, remain, nIDX, pgn, pgn0;  // unsigned xxx
	u32 dwCtrl;
	u32 m_CurrentFrameStartIdx = 0; 
	u32 m_NextFrameStartIdx = 0;  

    __le32        *ptr;
	u32 nId = nDMA_channel;

		///printk("@@@@ locate buffer_next %p [prev=%p/next=%p]   &buf->vb.queue= %p \n",
		///	buf, q->queue.prev,q->queue.next,  &buf->vb.queue);

		// fill P field half frame SG mapping entries
	Field_P = &dev->Field_P[nId];
	Field_B = &dev->Field_B[nId];

	pt = &dev->m_Page0;
	BUG_ON(NULL == pt || NULL == pt->cpu);
    FieldSize = Frame_size /2;

   /// printk("$$$$Fixed_SG_Mapping  nId %d  size %d  Video Frame %d\n", nId, FieldSize, Frame_size);
   /// printk("&&&&&&&&&&&&&&& Fill DMA table pt %p, length %d   startpage %d    L->length %d  \n",
	///	    pt, length, startpage, list->length);


    nIDX = 128;   ///85;
    //// pgn = 85;   /// FieldSize / 4096;
    pgn0 = (FieldSize + 4095) / 4096;
    pgn = TW68_buffer_pages( Frame_size /2) -1;  // page number for 1 field

    m_NextFrameStartIdx += pgn;
	ptr = pt->cpu + (2* nIDX * nId); 
	printk("-??????????????????-_pgtable--nId %d,  pgn0 %d   pgn %d    m_NextFrameStartIdx: %d \n ", nId, pgn0, pgn,  m_NextFrameStartIdx);

	sglist = Field_P ->sglist;
    nbytes = 0;
	remain = 0;
	for (i = 0; i < Field_P->n_pages; i++, sglist++)
	{
		    // switch to B
		    if ((nbytes + sglist->length) <= FieldSize)
			{
				remain = sglist->length;
			}
			else
			{
                remain = FieldSize - nbytes;
			}

			if (remain != 4096)  // 0x1000
			{
				/// printk("--#######@@@@@@@@@@@@@@@@@@@@_pgtable--nId %d     i=%d      remain=  %d   \n", nId, i, remain );
			}

			if (remain <=0)
				break;
					///goto FieldB;

				{

                    dwCtrl = 				(((DMA_STATUS_HOST_READY &0x3)<<30) |
											(((i==0)&1)<<29) |
											///(1<<29)|
											((m_CurrentFrameStartIdx & 0x7F) << 14)  |
											((m_NextFrameStartIdx    & 0xFF) << 21)  |
											 ((pgn >70)<<13) |			///   70
											(remain & 0x1FFF));	// size

						*(ptr++) = cpu_to_le32(dwCtrl);

	                    *(ptr++) = cpu_to_le32(sg_dma_address(sglist) - sglist->offset );  /// setup page dma address

						nbytes += sglist->length;

                }
	}

///FieldB:
        remain = 0;  /// (nbytes + sglist->length) - FieldSize;
        nbytes = 0;
		sglist = Field_B ->sglist;

        ptr = (pt->cpu + (2* nIDX * nId)) + 0x800;  // 2 pages distance  switch to B

		for (i = 0; i < Field_B->n_pages; i++, sglist++)
		{
		    // switch to B
		    if ((nbytes + sglist->length) <= FieldSize)
			{
				remain = sglist->length;
			}
			else
			{
                remain = FieldSize - nbytes;
			}
						
			if (remain != 4096)  // 0x1000
			{
				/// printk("--#######@@@@@@@@@@@@@@@@@@@@_pgtable--nId %d     i=%d      remain=  %d   \n", nId, i, remain );
			
			}

			if (remain <=0)
					break;

               /// if (remain >0)
                {

                    dwCtrl = 				(((DMA_STATUS_HOST_READY &0x3)<<30) |
											(((i==0)&1)<<29) |
											///(1<<29)|
											((m_CurrentFrameStartIdx & 0x7F) << 14)  |          /// 0xFF
											((m_NextFrameStartIdx    & 0xFF) << 21)  |
											 ((pgn >70)<<13) |
											(remain & 0x1FFF));	// size

						*(ptr++) = cpu_to_le32(dwCtrl);

	                    *(ptr++) = cpu_to_le32(sg_dma_address(sglist) - sglist->offset );  /// setup page dma address

						nbytes += sglist->length;   /// remain
                }
		}
  }

////////////////////////////////////////////////////BFDMA_setup(  );/////////////////////////////////////////////////////////////
void BFDMA_setup(struct TW68_dev *dev, int nDMA_channel, int H, int W)  //    Field0   P B    Field1  P B     WidthHightPitch
{
	u32  regDW, dwV, dn;

	    reg_writel((BDMA_ADDR_P_0 + nDMA_channel*8), dev->BDbuf[nDMA_channel][0].dma_addr);						//P DMA page table
		reg_writel((BDMA_ADDR_B_0 + nDMA_channel*8), dev->BDbuf[nDMA_channel][1].dma_addr);	
		reg_writel((BDMA_WHP_0 + nDMA_channel*8), (W& 0x7FF) | ((W& 0x7FF)<<11) | ((H &0x3FF)<<22));	

	    reg_writel((BDMA_ADDR_P_F2_0 + nDMA_channel*8), dev->BDbuf[nDMA_channel][2].dma_addr);						//P DMA page table
		reg_writel((BDMA_ADDR_B_F2_0 + nDMA_channel*8), dev->BDbuf[nDMA_channel][3].dma_addr);	
		reg_writel((BDMA_WHP_F2_0 + nDMA_channel*8), (W& 0x7FF) | ((W& 0x7FF)<<11) | ((H &0x3FF)<<22));	


    regDW = reg_readl( PHASE_REF_CONFIG );
	dn = (nDMA_channel<<1) + 0x10;
	dwV = (0x3 << dn);
	regDW |= dwV;
    reg_writel( PHASE_REF_CONFIG, regDW );
    dwV = reg_readl( PHASE_REF_CONFIG );

   //  printk(KERN_INFO "DMA mode setup %s: %d PHASE_REF_CONFIG  dn 0x%lx    0x%lx  0x%lx  H%d W%d  \n",  
	//		dev->name, nDMA_channel, regDW, dwV,  dn, H, W );


}

 ////////////////////////////////////////////////////////////////////////////////////////////////////////////


int Field_Copy(struct TW68_dev *dev, int nDMA_channel, int field_PB)
{
	struct TW68_dmaqueue *q;
	struct TW68_buf *buf = NULL; //,*next = NULL;
	int Hmax, Wmax, h, pos, pitch;

	struct dma_region			*Field_P;
	struct dma_region			*Field_B;

	/// printk(KERN_INFO " Field_Copy:  start 0000\n");
	int nId = nDMA_channel +1;

	void *vbuf, *srcbuf;  // = videobuf_to_vmalloc(&buf->vb);

		///printk("@@@@ locate buffer_next %p [prev=%p/next=%p]   &buf->vb.queue= %p \n",
		///	buf, q->queue.prev,q->queue.next,  &buf->vb.queue);

		// fill P field half frame SG mapping entries
	Field_P = &dev->Field_P[nDMA_channel];
	Field_B = &dev->Field_B[nDMA_channel];
	if (field_PB)
		srcbuf = Field_B->kvirt;
	else
		srcbuf = Field_P->kvirt;


	q = &dev->video_dmaq[nId];   ///  &dev->video_q;

	//  printk(KERN_INFO " nId%d   Field_Copy:  get DMA queue:%p,   curr %p\n", nId, q, q->curr);
	/*
	printk(KERN_INFO " Field_Copy:  get DMA queue list head:%p, \n", q->queue);
	printk(KERN_INFO " Field_Copy:  get DMA queue list head address:%p, \n", &q->queue);
	*/

	
	if (q->curr)
	{	
		buf = q->curr;
		vbuf = videobuf_to_vmalloc(&buf->vb);

		Hmax  = buf->vb.height/2;
		Wmax  = buf->vb.width;

		pitch = Wmax * buf->fmt->depth /8;
		pos = pitch * (field_PB);

		/// printk(KERN_INFO " Field_Copy:  start @@@@@@@@\n");   /// vbuf null

		for (h = 0; h < Hmax; h++)
		{
		memcpy(vbuf + pos, srcbuf, pitch);
		pos += pitch*2;
		srcbuf += pitch;
		}
		/// printk(KERN_INFO " Field_Copy:  Done %%%%%%%%%\n");
    }
    else
	{	
		 printk( " Field_Copy:::::::::     list_empty  \n");
		return 0;
	}
       return 1;
}


int BF_Copy(struct TW68_dev *dev, int nDMA_channel, u32 Fn, u32 PB)
{
	struct TW68_dmaqueue *q;
	struct TW68_buf *buf = NULL; //,*next = NULL;
	int n, Hmax, Wmax, h, pos, pitch;

	///struct dma_region			*Field_P;
	///struct dma_region			*Field_B;
	///struct scatterlist			*sglist;


	/// printk(KERN_INFO " Field_Copy:  start 0000\n");
	int nId = nDMA_channel +1;

	void *vbuf, *srcbuf;  // = videobuf_to_vmalloc(&buf->vb);

		///printk("@@@@ locate buffer_next %p [prev=%p/next=%p]   &buf->vb.queue= %p \n",
		///	buf, q->queue.prev,q->queue.next,  &buf->vb.queue);

		// fill P field half frame SG mapping entries
	pos = 0;
	n =0;
	if (Fn)
		n = 2;
	if (PB)
		n++;

	srcbuf = dev->BDbuf[nDMA_channel][n].cpu;

	q = &dev->video_dmaq[nId];   ///  &dev->video_q;

	//  printk(KERN_INFO " nId%d   Field_Copy:  get DMA queue:%p,   curr %p\n", nId, q, q->curr);
	/*
	printk(KERN_INFO " Field_Copy:  get DMA queue list head:%p, \n", q->queue);
	printk(KERN_INFO " Field_Copy:  get DMA queue list head address:%p, \n", &q->queue);
	*/

	
	if (q->curr)
	{	
		buf = q->curr;
		vbuf = videobuf_to_vmalloc(&buf->vb);

		Hmax  = buf->vb.height/2;
		Wmax  = buf->vb.width;

		pitch = Wmax * buf->fmt->depth /8;
		if (Fn)
			pos = pitch ;

		//printk(KERN_INFO " liutest Field_Copy:  start @@@@@@@@ Fn=%d ,PB=%d, srcbuf:%X    vbuf:%X   H %d   W %d  P %d\n",Fn,PB, srcbuf, vbuf, Hmax, Wmax, pitch);   /// vbuf null
		//printk(KERN_INFO " liutest Field_Copy:  start @@@@@@@@ ch=%d,Fn=%d ,PB=%d srcbuf:%X,vbuf:%X\n",nId,Fn,PB,srcbuf, vbuf);   /// vbuf null

		//vbuf += pitch * Hmax;
		//if (n==1)
		for (h = Hmax; h < Hmax + Hmax ; h++)
		{
		memcpy(vbuf + pos, srcbuf, pitch);
		pos += pitch *2;
		srcbuf += pitch;
		}
		
		//printk(KERN_INFO " liutest Field_Copy:  Done %%%%%%%%%\n");
    }
    else
	{	
		printk( " Block [][] Field_Copy:::::::::     list_empty  \n");
		return 0;
	}
       return 1;
}




int  QF_Field_Copy(struct TW68_dev *dev, int nDMA_channel, u32 Fn, u32 PB)
{
	struct TW68_dmaqueue *q;
	struct TW68_buf *buf = NULL;   ///,*next = NULL;
	int Hmax, Wmax, h, n, pos, pitch, stride;

	/// printk(KERN_INFO " Field_Copy:  start 0000\n");
	int nId = 0;

	void *vbuf, *srcbuf;  // = videobuf_to_vmalloc(&buf->vb);

/*
	field_PB = dev->video_dmaq[0].FieldPB & (1<<nDMA_channel);

	Field_P = &dev->Field_P[nDMA_channel];
	Field_B = &dev->Field_B[nDMA_channel];
	if (field_PB)
		srcbuf = Field_B->kvirt;
	else
		srcbuf = Field_P->kvirt;
*/

	n =0;
	if (Fn)
		n = 2;
	if (PB)
		n++;

	srcbuf = dev->BDbuf[nDMA_channel][n].cpu;

	q = (&dev->video_dmaq[nId]);   ///  &dev->video_q; (unsigned long)

	if (q->curr)
	{	
		buf = q->curr;
		Hmax  = buf->vb.height/2;
		Wmax  = buf->vb.width/2;

		pitch = 2* Wmax * buf->fmt->depth /8;
		stride = pitch /2;

		if (nDMA_channel ==0)
			pos =0; 
		if (nDMA_channel ==1)
			pos += stride;  
		if (nDMA_channel ==2)
			pos += pitch * Hmax;
		if (nDMA_channel ==3)
			pos += pitch * Hmax + stride;


		vbuf = videobuf_to_vmalloc(&buf->vb);


		for (h = 0; h < Hmax-0; h++) {
		memcpy(vbuf + pos, srcbuf, stride);
		pos += pitch;
		srcbuf += stride;

		}
       }
      else
	{	
		return 0;
	}
       return 1;

}


void DecoderResize(struct TW68_dev *dev, int nId, int nHeight, int nWidth)
{
	u32 nAddr, nHW, nH, nW, nVal, nReg, regDW;

	if(nId >=8 ) {
		/// printk( "DecoderResize() error: nId:%d,Width=%d,Height=%d\n", nId, nWidth, nHeight);
		return;
	}
	/// printk( "DecoderResize() ::::: nId:%d,Width=%d,Height=%d\n", nId, nWidth, nHeight);
		
	// only for internal 4     HDelay VDelay   etc
	nReg = 0xe7;   //  blue back color
	reg_writel(MISC_CONTROL2, nReg);	

	//fHeight = nHeight/2;  /// field H

	///if ((nHeight == 240) || (nHeight == 288))
	///	nWidth = 720;
	if (((nHeight == 240) || (nHeight == 288)) &&(nWidth > 700))
	   nWidth = 720;

	// initailize standard changes for different field height 

	nHW = nWidth | (nHeight<<16) | (1<<31);
	nH = nW = nHW;
	
	if(dev->PAL50[nId+1])
	{	
		//VDelay
		regDW = 0x18;
		if (nId <4)
		{
			nAddr = VDELAY0 + (nId *0x10);
			reg_writel(nAddr,  regDW);	

		}
		else
		{
			nAddr = VDELAY0 + ((nId-4) *0x10) + 0x100;
			reg_writel(nAddr,  regDW);	
		}

		//HDelay
		regDW = 0x0A;
		regDW = 0x0C;

		if (nId <4)
		{
			nAddr = HDELAY0 + (nId *0x10);
			reg_writel(nAddr,  regDW);	
		}
		else
		{
			nAddr = HDELAY0 + ((nId-4) *0x10) + 0x100;
			reg_writel(nAddr,  regDW);	

		}
	}
	else
	{
		//VDelay
		regDW = 0x14;
		if (nId <4)
		{
			nAddr = VDELAY0 + (nId *0x10);
			reg_writel(nAddr,  regDW);	

		}
		else
		{
			nAddr = VDELAY0 + ((nId-4) *0x10) + 0x100;
			reg_writel(nAddr,  regDW);	
		}

		//HDelay
		regDW = 0x0D;
		regDW = 0x0E;

		if (nId <4)
		{
			nAddr = HDELAY0 + (nId *0x10);
			reg_writel(nAddr,  regDW);	
		}
		else
		{
			nAddr = HDELAY0 + ((nId-4) *0x10) + 0x100;
			reg_writel(nAddr,  regDW);	
		}
	}


	///nVal = 0;  ///0x0a;
	///reg_writel(HDELAY0+nId, nVal);   // adjust start pixel
	nVal = reg_readl(HDELAY0+nId);
	printk( "DecoderResize() :: nId:%d, HDELAY=0x%X,  H:%d  W:%d   HW %X  HDELAY0 %X\n", nId, nVal, nHeight, nWidth, nHW, nVal);   /// read default  0x0a;

	// reg_writel (HACTIVE_L0+nId, 0xC0);  // 2C0  704

	//Video Size
	reg_writel(VIDEO_SIZE_REG,  nHW);	    //for Rev.A backward compatible
	reg_writel(VIDEO_SIZE_REG0+nId, nHW);   //for Rev.B or later only



	//decoder  Scaler 
	nH &= 0x7FF;
	nH = (720*256)/nH;  

	nW = (nW>>16)&0x1FF; 

	// PAL
	//nW = (288*256)/nW;
	// NTSC

	if (nHeight >240)   //PAL
	{
		nW = (288*256)/nW;
	}
	else				//NTSC
	{
		nW = (240*256)/nW;
	}

	if(nId >= 4)
	{
		nAddr = (EX_VSCALE1_LO +  ((nId-4)<<1) + (nId-4)) + 0x100; //EX_VSCALE1_LO + 0|3|6|9
		nAddr = VSCALE1_LO + ((nId-4)<<4) + 0x100; //6869 0x200 VSCALE1_LO + 0|0x10|0x20|0x30

	}
	else
		nAddr = VSCALE1_LO + (nId<<4); //VSCALE1_LO + 0|0x10|0x20|0x30


	nVal = nW & 0xFF;  //V

	if(nId >= 4) 
	{
		///DeviceWrite2864(dev, nAddr, (unsigned char)nVal);
		///nReg = DeviceRead2864(dev, nAddr);
	}
	else 
	{
		reg_writel(nAddr,  nVal);	    
		nReg = reg_readl(nAddr);	
	}

		reg_writel(nAddr,  nVal);	    
		nReg = reg_readl(nAddr);	

	nAddr++;  //V H
	nVal = (((nW >> 8) & 0xF) << 4) | (( nH >> 8) & 0xF ); 

	nAddr++;  //H
	nVal = nH & 0xFF;


	if(nId >= 4) 
	{
		///DeviceWrite2864(dev, nAddr, (unsigned char)nVal);
		///nReg = DeviceRead2864(dev, nAddr);
	}
	else 
	{
		reg_writel(nAddr,  nVal);	    
		nReg = reg_readl(nAddr);	
	}

		reg_writel(nAddr,  nVal);	    
		nReg = reg_readl(nAddr);	

	//// printk( "DecoderResize() ?????????????????:: nId:%d,    nAddr%X  nReg:%X\n", nId, nAddr, nReg);


	nAddr++;  //H
	nVal = nH & 0xFF;

	if(nId >= 4) 
	{
		///DeviceWrite2864(dev, nAddr, (unsigned char)nVal);
		///nReg = DeviceRead2864(dev, nAddr);
	}
	else 
	{
		reg_writel(nAddr,  nVal);	    
		nReg = reg_readl(nAddr);	
	}

		reg_writel(nAddr,  nVal);	    
		nReg = reg_readl(nAddr);	

// H Scaler
	nVal = (nWidth-12 - 4)*(1<<16)/nWidth;
	nVal = (4 & 0x1F) |
		  (((nWidth-12) & 0x3FF) << 5) | 
		  (nVal <<15);

	// reg_writel(SHSCALER_REG0+nId, nVal);	

}



/////--------------------------------DMA start etc--------------------------------------------------

void resync(unsigned long data)
{
	struct TW68_dev *dev = (struct TW68_dev*)data;
	u32 dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF, Reg8b, nId, k, m;
	unsigned long flags;
	unsigned long	now = jiffies;

	//printk("liutest resync!\n");

	mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(50));
	if ( now - dev->errlog[0] < msecs_to_jiffies(50))
	{
		///printk(" resync _reset_ sync time now%lX - errlog %lX \n", now, dev->errlog[0]);
		return;
	}

	if 	(dev->videoDMA_ID ==0)
	{
		dwRegE = 0; //~(0xFF);
        reg_writel(DMA_CHANNEL_ENABLE,dwRegE);
		dwRegF = (1<<31);
		dwRegF = 0;
		dwRegF |= dwRegE;
		reg_writel(DMA_CMD, dwRegF);
        ///printk(" resync _reset_ sync field order on 6864B :   DMA_CHANNEL_ENABLE %x   \n", dwRegE);
		// printk(" resync _reset_ sync field : DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);
	}

	spin_lock_irqsave(&dev->slock, flags);
	dwRegST = reg_readl(DMA_INT_STATUS);
    dwRegER = reg_readl(DMA_INT_ERROR);
    dwRegPB = reg_readl(DMA_PB_STATUS);
    dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
	dwRegF = reg_readl(DMA_CMD);
	spin_unlock_irqrestore(&dev->slock, flags);

	//	printk(" resync _reset_ sync field : DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  			DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);
        
	///if(dwRegER  || (dwRegST >>24))   //stopped?
	{
		m = 0;
		Reg8b = 0;
		if (dev->videoDMA_ID != (dwRegE& 0xFF))
		{
			Reg8b = (dev->videoDMA_ID ^ dwRegE) & dev->videoDMA_ID;
			for ( k =0; k<8; k++)
			{
				if (Reg8b &(1<<k))
				{
				   dwRegE |= Reg8b &(1<<k);
				   m++;
				   //k = 8;
				}
				if (m >2) break;
			}

		spin_lock_irqsave(&dev->slock, flags);

        reg_writel(DMA_CHANNEL_ENABLE,dwRegE);
		dwRegF = (1<<31);
		dwRegF |= dwRegE;
		reg_writel(DMA_CMD, dwRegF);
		spin_unlock_irqrestore(&dev->slock, flags);

        ///printk(" resync _reset_ sync field order on 6864B :   DMA_CHANNEL_ENABLE %x   \n", dwRegE);
		printk(" resync _reset_ sync @ Enable :  Reg8b:0x%X  DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  	DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", 
			   Reg8b, dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);

		dev->video_DMA_1st_started += 1;   //++
		}
		else
		for (nId=0; nId<8; nId++)
		{
			if (nId<4)
				Reg8b = reg_readl(DECODER0_STATUS + (nId* 0x10));
			///else
				///Reg8b = DeviceRead2864(dev, VIDEO_STATUS_0 + ((nId -4)* 0x10));

			if ( Reg8b & 0x80 )    // VDLOSS
				dev->videoDecoderST[nId] = 1;
			else
			{
				if (dev->videoDecoderST[nId])
				{// status 2   regain signal
					// stop   nId
					
					dwRegE &= ~(1 << nId);
					reg_writel(DMA_CHANNEL_ENABLE, dwRegE);
					///dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
					dwRegF &= ~(1 << nId);
					reg_writel(DMA_CMD, dwRegF);
					///dwRegF = reg_readl(DMA_CMD);
					printk(" resync _reset_ sync @ VDLOSS:%d :  Reg8b:0x%X  DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  	DMA_CHANNEL_ENABLE %x   \n", 
											 nId, Reg8b, dwRegST, dwRegER, dwRegPB, dwRegE);
				}
				dev->videoDecoderST[nId] = 0;
			}
		}
	}
}


// special treatment for intel MB
u64 GetDelay(struct TW68_dev *dev, int eno)
{
	u64 	delay, k, last, now, pause;
	last =0;
	pause = 40;  // 20  30  50

/*	
	if (eno > 1)
		pause = 30;     

	if (eno > 3)	
		pause = 60;
*/
		
	/// printk("\n $$$$$$$$$$$$$$$$ go through timers:   jiffies:0x%X ", jiffies);
	for (k=0; k<8; k++)
	{
		/// delay = dev->video_dmaq[k].restarter.expires;
		///printk( " 0X%x ", delay);
		// last = (delay > last) ? delay : last;
 	}

	///delay = (last >jiffies) ? last : jiffies;
	///printk( " last 0X%x   jiffies 0X%x ", last, jiffies);
	now = jiffies;
	if (last > now)
		delay = last;
	else
		delay = jiffies;

	///printk(" delay:0x%X  ", delay);
	if ((delay == jiffies)&& ((last + msecs_to_jiffies(pause)) >delay))
		delay = last + msecs_to_jiffies(pause);
	///printk(" delay:0x%X  ", delay);

	delay += msecs_to_jiffies(pause);

	///printk("\n $$$$$$$$$$$$$$$$ search delay :  40 msec %d        last:0X%x    jiffies:0X%x    delay:0X%x   \n", msecs_to_jiffies(40),  last, jiffies, delay );
	return delay;
}



void TW68_buffer_timeout(unsigned long data)
{
   u32 dwRegE, dwRegF;

	struct TW68_dmaqueue *q = (struct TW68_dmaqueue*)data;
	struct TW68_dev *dev = q->dev;
	///unsigned long flags;
	int nId = q->DMA_nCH;

	printk(" liutest 1 TW68_buffer_timeout ????????  DMA %d  || 0x%X  ||0X%X     timeout on dma queue %p\n", nId,  dwRegE, dwRegF, q->curr);
	
	if (q->curr) {
		
	dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
	dwRegF = reg_readl(DMA_CMD);

	printk(" TW68_buffer_timeout ????????  DMA %d  || 0x%X  ||0X%X     timeout on dma queue %p\n", nId,  dwRegE, dwRegF, q->curr);
	TW68_buffer_finish(dev,q,VIDEOBUF_ERROR);
	//TW68_buffer_next(dev,q);  //liutest
	}
	printk(" liutest 2 TW68_buffer_timeout ????????  DMA %d  || 0x%X  ||0X%X     timeout on dma queue %p\n", nId,  dwRegE, dwRegF, q->curr);
	TW68_buffer_next(dev,q);  // liutest move to finish
}

/* ------------------------------------------------------------------ */

int TW68_set_dmabits(struct TW68_dev *dev,  unsigned int DMA_nCH)
{
	u32  dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF, nId, k, run;
	nId = DMA_nCH;   ////

	dwRegST = reg_readl(DMA_INT_STATUS);

    dwRegER = reg_readl(DMA_INT_ERROR);

    dwRegPB = reg_readl(DMA_PB_STATUS);

    dwRegE = reg_readl(DMA_CHANNEL_ENABLE);

	dwRegF = reg_readl(DMA_CMD);

		/// printk(" _dmabits _set_dmabits : DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);

	dev->video_DMA_1st_started += 1;   //++
	dev->video_dmaq[DMA_nCH].FieldPB = 0;

			
	dwRegE |= (1 << nId);  // +1 + 0  Fixed PB

	dev->videoCap_ID |= (1 << nId) ; 
	dev->videoDMA_ID |= (1 << nId) ; 
    reg_writel(DMA_CHANNEL_ENABLE,dwRegE);

	run = 0;
	
	for (k=0; k<8; k++)
	{
	 if (run < dev->videoDMA_run[k])
		run = dev->videoDMA_run[k];
	}
	
	dev->videoDMA_run[nId] = run+1;
	dwRegF = (1<<31);
	dwRegF |= dwRegE;
	reg_writel(DMA_CMD, dwRegF);
		///dwRegF = reg_readl(DMA_CMD);
		/// printk(" set_dmabits ?????????????? _set_dmabits : DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegE, dwRegF);
	/// printk(" TW68_set_dmabits _set_dmabits : DMA_INT_STATUS%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);
	return 0;
}

/////////////////////////////////////////////////////////////

int	stop_video_DMA(struct TW68_dev *dev, unsigned int DMA_nCH)
{
	u32  dwRegER, dwRegPB, dwRegE, dwRegF, nId;
	nId = DMA_nCH;  ///2;

    dwRegER = reg_readl(DMA_INT_ERROR);
    dwRegPB = reg_readl(DMA_PB_STATUS);
    dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
	dwRegF = reg_readl(DMA_CMD);

	dwRegE &= ~(1 << nId);
    reg_writel(DMA_CHANNEL_ENABLE, dwRegE);
    dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
	dev->videoDMA_ID &= ~(1 << nId);
	dev->videoCap_ID &= ~(1 << nId);

	dwRegF &= ~(1 << nId);
    reg_writel(DMA_CMD, dwRegF);
    dwRegF = reg_readl(DMA_CMD);

	dev->videoDMA_run[nId] = 0;

	if 	(dev->videoCap_ID ==0)
	{
		reg_writel(DMA_CMD, 0);
		reg_writel(DMA_CHANNEL_ENABLE, 0);

	}

    /// printk(KERN_INFO " -------------stop_video_DMA   DMA_CHANNEL_ENABLE 0x%X,  DMA_CMD 0x%X,  DMA_INT_STATUS 0x%08X \n",
	///             dwRegE, dwRegF,  dwRegST      );
	return 0;
}


int	VideoDecoderDetect(struct TW68_dev *dev, unsigned int DMA_nCH)
{
		u32	regDW, dwReg;

		if (DMA_nCH <4)  // VD 1-4
		{
				regDW = reg_readl(DECODER0_STATUS + (DMA_nCH* 0x10));
				dwReg = reg_readl(DECODER0_SDT + (DMA_nCH* 0x10));
		}
		else		// 6869  VD 5-8
		{
				//regDW = (ULONG)DeviceRead2864(this, VIDEO_STATUS_0 + ((nID -4)* 0x10));
				//dwReg = (ULONG)DeviceRead2864(this, exVD0_SDT + ((nID -4)* 0x10));
				regDW = reg_readl(DECODER0_STATUS + ((DMA_nCH -4)* 0x10) + 0x100);
				dwReg = reg_readl(DECODER0_SDT + ((DMA_nCH -4)* 0x10) + 0x100);
				///printk("\n\n Decoder 0x%X VideoStandardDetect DMA_nCH %d  regDW 0x%x  dwReg%d \n", (DECODER0_STATUS + (DMA_nCH* 0x10) + 0x100), regDW, dwReg );
		}

		if (( regDW &1)) //&& (!(dwReg & 0x80)))   ///skip the detection glitch     //detect properly
		{
			// set to PAL 50 for real...
			// VDelay
			printk("50HZ VideoStandardDetect DMA_nCH %d  regDW 0x%x  dwReg%d \n", DMA_nCH, regDW, dwReg );
			return 50;
		}
		else
		{
			printk("60HZ VideoStandardDetect DMA_nCH %d  regDW 0x%x  dwReg%d \n", DMA_nCH, regDW, dwReg );
			return 60;
			
		}
}

// liutest add reysnc tasklet
static void
resync_tasklet(unsigned long data)
{
	struct TW68_dev *dev = (struct TW68_dev*)data;
	u32 dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF, Reg8b, nId, k, m;
	unsigned long flags;
	unsigned long	now = jiffies;

	//printk("liutest resync!\n");

	//mod_timer(&dev->delay_resync, jiffies+ msecs_to_jiffies(50));
	//if ( now - dev->errlog[0] < msecs_to_jiffies(50))
	//{
	//	///printk(" resync _reset_ sync time now%lX - errlog %lX \n", now, dev->errlog[0]);
	//	return;
	//}

	if 	(dev->videoDMA_ID ==0)
	{
		dwRegE = 0; //~(0xFF);
        reg_writel(DMA_CHANNEL_ENABLE,dwRegE);
		dwRegF = (1<<31);
		dwRegF = 0;
		dwRegF |= dwRegE;
		reg_writel(DMA_CMD, dwRegF);
        ///printk(" resync _reset_ sync field order on 6864B :   DMA_CHANNEL_ENABLE %x   \n", dwRegE);
		// printk(" resync _reset_ sync field : DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);
	}

	spin_lock_irqsave(&dev->slock, flags);
	dwRegST = reg_readl(DMA_INT_STATUS);
    dwRegER = reg_readl(DMA_INT_ERROR);
    dwRegPB = reg_readl(DMA_PB_STATUS);
    dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
	dwRegF = reg_readl(DMA_CMD);
	spin_unlock_irqrestore(&dev->slock, flags);

	//	printk(" resync _reset_ sync field : DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  			DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);
        
	///if(dwRegER  || (dwRegST >>24))   //stopped?
	{
		m = 0;
		Reg8b = 0;
		if (dev->videoDMA_ID != (dwRegE& 0xFF))
		{
			Reg8b = (dev->videoDMA_ID ^ dwRegE) & dev->videoDMA_ID;
			for ( k =0; k<8; k++)
			{
				if (Reg8b &(1<<k))
				{
				   dwRegE |= Reg8b &(1<<k);
				   m++;
				   //k = 8;
				}
				if (m >2) break;
			}

			spin_lock_irqsave(&dev->slock, flags);

			reg_writel(DMA_CHANNEL_ENABLE,dwRegE);
			dwRegF = (1<<31);
			dwRegF |= dwRegE;
			reg_writel(DMA_CMD, dwRegF);
			spin_unlock_irqrestore(&dev->slock, flags);

			///printk(" resync _reset_ sync field order on 6864B :   DMA_CHANNEL_ENABLE %x   \n", dwRegE);
			printk(" resync _reset_ sync @ Enable :  Reg8b:0x%X  DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  	DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", 
			   Reg8b, dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);

			dev->video_DMA_1st_started += 1;   //++
		}
		else
		for (nId=0; nId<8; nId++)
		{
			if (nId<4)
				Reg8b = reg_readl(DECODER0_STATUS + (nId* 0x10));
			///else
				///Reg8b = DeviceRead2864(dev, VIDEO_STATUS_0 + ((nId -4)* 0x10));

			if ( Reg8b & 0x80 )    // VDLOSS
				dev->videoDecoderST[nId] = 1;
			else
			{
				if (dev->videoDecoderST[nId])
				{// status 2   regain signal
					// stop   nId
					
					dwRegE &= ~(1 << nId);
					reg_writel(DMA_CHANNEL_ENABLE, dwRegE);
					///dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
					dwRegF &= ~(1 << nId);
					reg_writel(DMA_CMD, dwRegF);
					///dwRegF = reg_readl(DMA_CMD);
					printk(" resync _reset_ sync @ VDLOSS:%d :  Reg8b:0x%X  DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  	DMA_CHANNEL_ENABLE %x   \n", 
											 nId, Reg8b, dwRegST, dwRegER, dwRegPB, dwRegE);
				}
				dev->videoDecoderST[nId] = 0;
			}
		}
	}
}

// liutest add video buff tasklet
static void
videobuf_tasklet(unsigned long data)
{
    struct TW68_dev* dev = (struct TW68_dev*) data;
    unsigned long flags;
	int k;

				  for(k=0; k<8; k++)  {
						if ((dev->dwRegST & dev->videoDMA_ID) & ( 1 << k) )  {   /// exclude  inactive dev
					

							TW68_irq_video_done(dev, k+1, dev->dwRegPB);

							if (!dev->video_dmaq[k+1].FieldPB) {   // first time after dma start
							
							/*
							Reg8b <<= 16;
							dev->video_dmaq[k+1].FieldPB &= 0x0000FFFF;
							dev->video_dmaq[k+1].FieldPB |= Reg8b;	// add
							/// printk(" IRQ DMA normal ist time  k:%d  Reg8b 0x%x: PB 0x%x  FieldPB 0X%X DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n",
							///			k, Reg8b, dwRegPB, dev->video_dmaq[k].FieldPB,  dwRegE, dwRegF);

								if (Reg8b &0x10)
									dev->video_dmaq[k].FieldPB |= 0xF0;
							*/
							}

							if  (dev->video_dmaq[k+1].FieldPB & 0xF0) {
							
								dev->video_dmaq[k+1].FieldPB &= 0xFFFF0000;
							}
							else {

								dev->video_dmaq[k+1].FieldPB &= 0xFFFF00FF;     // clear  PB
								dev->video_dmaq[k+1].FieldPB |= (dev->dwRegPB & (1<<k))<<8;
								dev->video_dmaq[k+1].FCN++;
							}
						}
				  }
			
}

/* ------------------------------------------------------------------ */

static irqreturn_t TW68_irq(int irq, void *dev_id)    /// hardware dev id for the ISR
{	
	struct TW68_dev *dev = (struct TW68_dev*) dev_id;
	unsigned long flags, audio_ch, k, eno, handled;
    u32 dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF, dwRegVP, dwErrBit;
	static int INT1st = 1;
	static u32 lastPB = 0;

    // __le32  *pdmaP, *pdmaB;
    audio_ch = 1;
	handled = 1;

	spin_lock_irqsave(&dev->slock, flags);

	dwRegST = reg_readl(DMA_INT_STATUS);
    dwRegER = reg_readl(DMA_INT_ERROR);
    dwRegPB = reg_readl(DMA_PB_STATUS);
	dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
	dwRegVP = reg_readl(VIDEO_PARSER_STATUS);
	dwRegF = reg_readl(DMA_CMD);
	spin_unlock_irqrestore(&dev->slock, flags);

	if (dev->videoRS_ID)
     printk("---%X+++++++++ IRQ: DMA_INT_STATUS 0X%X  DMA_FIFO_ERROR%X  DMA_PB_STATUS%X  DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n", 
				dev->videoRS_ID, dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF);

	///////////////////////////////////////////////////////////////////////////
		if 	(dev->videoDMA_ID != dwRegE)
		{
			INT1st++;
			if (INT1st <10)
			printk(KERN_INFO " -------------Check DMA   DMA_CHANNEL_ENABLE 0x%X,  DMA_CMD 0x%X,  DMA_INT_STATUS 0x%08X   videoDMA_ID 0x%X\n",
			dwRegE, dwRegF,  dwRegST,   dev->videoDMA_ID    );

		}

	/*
	for (audio_ch =0; audio_ch < MAX_NUM_DATA_DMA; audio_ch++)
	{
		///pdmaP = dev->m_AudioBuffer.cpu + (PAGE_SIZE <<1)* audio_ch + 100;
		///pdmaB = dev->m_AudioBuffer.cpu + (PAGE_SIZE <<1)* audio_ch + PAGE_SIZE + 100;

		pdmaP = dev->m_AudioBuffer.cpu + (PAGE_SIZE <<1)* audio_ch /4 + 100;
		pdmaB = dev->m_AudioBuffer.cpu + ((PAGE_SIZE <<1)* audio_ch + PAGE_SIZE)/4 + 100;
		/// printk("---Audio DMA 0x19-0x29 config --- CH%02d    *dmaP 0X%x  *dmaB 0X%x  dword 0X%x 0X%x   \n",
		///	    audio_ch, pdmaP, pdmaB, *pdmaP, *pdmaB );
	}
	*/

    if((dwRegER &0xFF000000) && dev->video_DMA_1st_started && dev->err_times<9)
	{
		dev->video_DMA_1st_started --;
		if (dev->video_DMA_1st_started < 0)
			dev->video_DMA_1st_started = 0;

		dev->err_times++;
		printk("DeviceInterrupt: 1st startup err_times:%d ## dma_status (err) =0x%x   dwRegVP (video parser)=0X%x   int_status 0x%x   dwRegE 0X%x \n", dev->err_times, dwRegER, dwRegVP, dwRegST, dwRegE);
	}
	else
	{
		if((dwRegER>>16 ) || dwRegVP || (dwRegST >>24))   //stop
		{ //err_exist, such as cpl error, tlp error, time-out
				dwErrBit = 0;
				dwErrBit |= ((dwRegST >>24) & 0xFF);
				dwErrBit |= (((dwRegVP >>8) |dwRegVP )& 0xFF);
				///dwErrBit |= (((dwRegER >>24) |(dwRegER >>16) |(dwRegER ))& 0xFF);
				dwErrBit |= (((dwRegER >>24) |(dwRegER >>16))& 0xFF);

		  eno =0;
		  for(k=0; k<8; k++)
		  {
			if ( dwErrBit & ( 1 << k) )
			{
				eno++;
				// Disable DMA channel
				dwRegE &= ~((1<< k));
				if (eno >2)
					dwRegE &= ~((0xFF));
			}
		  }

		  // stop  the error channel
  	  	  spin_lock_irqsave(&dev->slock, flags);

		  reg_writel(DMA_CHANNEL_ENABLE, dwRegE);
 		  dwRegF = (1<<31);
		  dwRegF |= dwRegE;
		  reg_writel(DMA_CMD, dwRegF);

		  spin_unlock_irqrestore(&dev->slock, flags);
	
		   printk("DeviceInterrupt: errors  ## dma_status  0x%X   (err) =0x%X   dwRegVP (video parser)=0X%x   int_status 0x%x  # dwRegE 0X%x dwRegF 0X%x \n", 
		   			dwErrBit, dwRegER, dwRegVP, dwRegST, dwRegE, dwRegF);
			dev->errlog[0] =  jiffies;

		}
		else
		{
		 // Normal interrupt:
	     // printk("  Normal interrupt:  ++++ ## dma_status 0x%X   FIFO =0x%X   (video parser)=0X%x   int_status 0x%x  PB 0x%x  # dwRegE 0X%x dwRegF 0X%x \n", 
		 //	     dwRegST, dwRegER, dwRegVP,  dwRegST, dwRegPB, dwRegE, dwRegF);

			// liutest add resync tasklet
			if (dwRegER & 0xFF)
				tasklet_schedule(&dev->resync_tasklet);

			if (lastPB != dwRegPB) {   // skip the conflict interrupt
			  if ((dwRegST & (0xFF)) && (!(dwRegER >>16)))
			  {
#if 0  // liutest add tasklet
				  for(k=0; k<8; k++)
				  {
					if ((dwRegST & dev->videoDMA_ID) & ( 1 << k) )     /// exclude  inactive dev
					{

							TW68_irq_video_done(dev, k+1, dwRegPB);

							if (!dev->video_dmaq[k+1].FieldPB)   // first time after dma start
							{
							/*
							Reg8b <<= 16;
							dev->video_dmaq[k+1].FieldPB &= 0x0000FFFF;
							dev->video_dmaq[k+1].FieldPB |= Reg8b;	// add
							/// printk(" IRQ DMA normal ist time  k:%d  Reg8b 0x%x: PB 0x%x  FieldPB 0X%X DMA_CHANNEL_ENABLE %x  DMA_CMD %X \n",
							///			k, Reg8b, dwRegPB, dev->video_dmaq[k].FieldPB,  dwRegE, dwRegF);

								if (Reg8b &0x10)
									dev->video_dmaq[k].FieldPB |= 0xF0;
							*/
							}

							if  (dev->video_dmaq[k+1].FieldPB & 0xF0)
							{
								dev->video_dmaq[k+1].FieldPB &= 0xFFFF0000;
							}
							else
							{

								dev->video_dmaq[k+1].FieldPB &= 0xFFFF00FF;     // clear  PB
								dev->video_dmaq[k+1].FieldPB |= (dwRegPB & (1<<k))<<8;
								dev->video_dmaq[k+1].FCN++;
							}
					}
				  }
#else
				dev->dwRegST =  dwRegST;
				dev->dwRegPB = dwRegPB;
				tasklet_schedule(&dev->bf_tasklet);
#endif
			  }
			  lastPB =  dwRegPB;  // liutest add
		  }
		}
	}
    if (!dwRegER && !dwRegST)  // skip the  interrupt  conflicts
	{
		//need to stop it
		/*
		  reg_writel(DMA_CHANNEL_ENABLE, 0);
			dwRegE = reg_readl(DMA_CHANNEL_ENABLE);

		  reg_writel(DMA_CMD, 0);
			dwRegF = reg_readl(DMA_CMD);
		*/

		if 	(dev->videoCap_ID ==0)
		{
			reg_writel(DMA_CMD, 0);
			reg_writel(DMA_CHANNEL_ENABLE,0);
			dwRegE = reg_readl(DMA_CHANNEL_ENABLE);
			dwRegF = reg_readl(DMA_CMD);
		}

        handled = 0;
		/// printk("---+++++++++ skip IRQ: DMA_INT_STATUS 0X%X  DMA_INT_ERROR%X  DMA_PB_STATUS%X  DMA_CHANNEL_ENABLE %x  DMA_CMD %X  k:%x\n", dwRegST, dwRegER, dwRegPB, dwRegE, dwRegF, k);
	}

	return IRQ_RETVAL(handled);
}


/* ------------------------------------------------------------------ */

/* early init (no i2c, no irq) */

static int TW68_hwinit1(struct TW68_dev *dev)
{
	u32  m_StartIdx, m_EndIdx, m_nVideoFormat, m_dwCHConfig,   dwReg, \
		 m_bHorizontalDecimate=0, m_bVerticalDecimate=0, m_nDropChannelNum,   \
		 m_bDropMasterOrSlave, m_bDropField, m_bDropOddOrEven, m_nCurVideoChannelNum;

	u32	 regDW, val1, addr, k, ChannelOffset, pgn;

	// Audio P
	int audio_ch;
	u32 dmaP, dmaB;

	printk(" TW6869 hwinit1 \n");
	mutex_init(&dev->lock);
	spin_lock_init(&dev->slock);

	/////////////////////////////////////////////////////////////////////////

    pci_read_config_dword( dev->pci, PCI_VENDOR_ID, &regDW);

    // printk(KERN_INFO "%s: found with ID: 0x%lx\n", dev->name,  regDW );

    pci_read_config_dword( dev->pci, PCI_COMMAND, &regDW);   // 04 PCI_COMMAND
	printk(KERN_INFO "%s: CFG[0x04] PCI_COMMAND :  0x%x\n", dev->name,  regDW );

	regDW |=  7;
	regDW &=  0xfffffbff;
	pci_write_config_dword( dev->pci, PCI_COMMAND, regDW);
    pci_read_config_dword( dev->pci, 0x4, &regDW);
    //  printk(KERN_INFO "%s: CFG[0x04]   0x%lx\n", dev->name,  regDW );

     pci_read_config_dword( dev->pci, 0x3c, &regDW);
     // printk(KERN_INFO "%s: CFG[0x3c]   0x%lx\n", dev->name,  regDW );

	  // MSI CAP     disable MSI
     pci_read_config_dword( dev->pci, 0x50, &regDW);
     	regDW &=  0xfffeffff;
	pci_write_config_dword( dev->pci, 0x50, regDW);
    pci_read_config_dword( dev->pci, 0x50, &regDW);
    //  printk(KERN_INFO "%s: CFG[0x50]   0x%lx\n", dev->name,  regDW );
	//  MSIX  CAP    disable
     pci_read_config_dword( dev->pci, 0xac, &regDW);
      	regDW &=  0x7fffffff;
	pci_write_config_dword( dev->pci, 0xac, regDW);
    pci_read_config_dword( dev->pci, 0xac, &regDW);
    //  printk(KERN_INFO "%s: CFG[0xac]   0x%lx\n", dev->name,  regDW );

	  // PCIe Cap registers
     pci_read_config_dword( dev->pci, 0x70, &regDW);
     // printk(KERN_INFO "%s: CFG[0x70]   0x%lx\n", dev->name,  regDW );
     pci_read_config_dword( dev->pci, 0x74, &regDW);
     // printk(KERN_INFO "%s: CFG[0x74]   0x%lx\n", dev->name,  regDW );

     pci_read_config_dword( dev->pci, 0x78, &regDW);
          	regDW &=  0xfffffe1f;
            regDW |=  (0x8 <<5);        ///  8 - 128   ||  9 - 256  || A - 512
	pci_write_config_dword( dev->pci, 0x78, regDW);

    pci_read_config_dword( dev->pci, 0x78, &regDW);
    //  printk(KERN_INFO "%s: CFG[0x78]   0x%lx\n", dev->name,  regDW );


    pci_read_config_dword( dev->pci, 0x730, &regDW);
    //  printk(KERN_INFO "%s: CFG[0x730]   0x%lx\n", dev->name,  regDW );

    pci_read_config_dword( dev->pci, 0x734, &regDW);
    //  printk(KERN_INFO "%s: CFG[0x734]   0x%lx\n", dev->name,  regDW );

    pci_read_config_dword( dev->pci, 0x738, &regDW);
    //  printk(KERN_INFO "%s: CFG[0x738]   0x%lx\n", dev->name,  regDW );



 	mdelay(20);
    reg_writel(DMA_CHANNEL_ENABLE, 0);
	mdelay(50);
    reg_writel(DMA_CMD, 0);
	
    reg_readl(DMA_CHANNEL_ENABLE);
    reg_readl(DMA_CMD);


	//Trasmit Posted FC credit Status
	reg_writel(EP_REG_ADDR, 0x730);   //
	regDW = reg_readl( EP_REG_DATA );
    //printk(KERN_INFO "%s: PCI_CFG[Posted 0x730]= 0x%lx\n", dev->name,  regDW );

	//Trasnmit Non-Posted FC credit Status
	reg_writel(EP_REG_ADDR, 0x734);   //
	regDW = reg_readl( EP_REG_DATA );
    //printk(KERN_INFO "%s: PCI_CFG[Non-Posted 0x734]= 0x%lx\n", dev->name,  regDW );

	//CPL FC credit Status
	reg_writel(EP_REG_ADDR, 0x738);   //
	regDW = reg_readl( EP_REG_DATA );
    //printk(KERN_INFO "%s: PCI_CFG[CPL 0x738]= 0x%lx\n", dev->name,  regDW );


	/////////////////////////////////////////////////////////////////////////////////////////////

    regDW = reg_readl( (SYS_SOFT_RST) );
    /// printk(KERN_INFO "HWinit %s: SYS_SOFT_RST  0x%lx    \n",  dev->name, regDW );
    ///regDW = tw_readl( SYS_SOFT_RST );
    ///printk(KERN_INFO "DMA %s: SYS_SOFT_RST  0x%lx    \n",  dev->name, regDW );

	reg_writel((SYS_SOFT_RST), 0x01);   //??? 01   09
	reg_writel((SYS_SOFT_RST), 0x0F);
    regDW = reg_readl( SYS_SOFT_RST );
    ///printk(KERN_INFO " After software reset DMA %s: SYS_SOFT_RST  0x%lx    \n",  dev->name, regDW );

    regDW = reg_readl( PHASE_REF_CONFIG );
    ///printk(KERN_INFO "HWinit %s: PHASE_REF_CONFIG  0x%lx    \n",  dev->name, regDW );
	regDW = 0x1518;
	reg_writel( PHASE_REF_CONFIG, regDW&0xFFFF );
	

	//  Allocate PB DMA pagetable  total 16K  filled with 0xFF
    videoDMA_pgtable_alloc(dev->pci, &dev->m_Page0);
	AudioDMA_PB_alloc(dev->pci, &dev->m_AudioBuffer);


	for ( k =0; k<8; k++)
	{
		if (dma_field_alloc(&dev->Field_P[k], 720*300*2, dev->pci, PCI_DMA_BIDIRECTIONAL))
		{
			/// printk(KERN_ERR,  "Failed to allocate dma buffer");
			dma_field_free(&dev->Field_P[k]);
			return -1;
		}

		if (dma_field_alloc(&dev->Field_B[k], 720*300*2, dev->pci, PCI_DMA_BIDIRECTIONAL))
		{
			/// printk(KERN_ERR, "Failed to allocate dma buffer");
			dma_field_free(&dev->Field_B[k]);
			return -1;
		}
	}

            ChannelOffset = pgn = 128;   ///125;
            pgn = 85;					///   starting for 720 * 240 * 2
            m_nDropChannelNum = 0;
             m_bDropMasterOrSlave = 1;   // master
              m_bDropField = 0;
               m_bDropOddOrEven = 0;

//	m_nVideoFormat = VIDEO_FORMAT_RGB565;
   m_nVideoFormat = VIDEO_FORMAT_YUYV;
   for (k = 0; k <MAX_NUM_SG_DMA; k++)
   {
		m_StartIdx = ChannelOffset * k;
		m_EndIdx = m_StartIdx + pgn;
		m_nCurVideoChannelNum = 0;  // real-time video channel  starts 0
		m_nVideoFormat = 0;   ///0; ///VIDEO_FORMAT_UYVY;

		m_dwCHConfig                   = ( m_StartIdx&0x3FF)			|    // 10 bits
										 ((m_EndIdx&0x3FF)<<10)			|	 // 10 bits
										 ((m_nVideoFormat&7)<<20)		|
										 ((m_bHorizontalDecimate&1)<<23)|
										 ((m_bVerticalDecimate&1)<<24)	|
										 ((m_nDropChannelNum&3)<<25)	|
										 ((m_bDropMasterOrSlave&1)<<27) |    // 1 bit
										 ((m_bDropField&1)<<28)			|
										 ((m_bDropOddOrEven&1)<<29)		|
										 ((m_nCurVideoChannelNum&3)<<30);

		reg_writel( DMA_CH0_CONFIG+ k,m_dwCHConfig);
		dwReg =  reg_readl(DMA_CH0_CONFIG+ k);
		/// printk(" ********#### buffer_setup%d::  m_StartIdx 0X%x  0x%X  dwReg: 0x%X  m_dwCHConfig 0x%X  \n", k, m_StartIdx, pgn,  m_dwCHConfig, dwReg );

	reg_writel( VERTICAL_CTRL, 0x24); //0x26 will cause ch0 and ch1 have dma_error.  0x24
	reg_writel( LOOP_CTRL,   0xA5  );     // 0xfd   0xA5     /// 1005
	reg_writel( DROP_FIELD_REG0+ k, 0);  ///m_nDropFiledReg
   }


	for (audio_ch =0; audio_ch < MAX_NUM_DATA_DMA; audio_ch++)
	{
		dmaP = dev->m_AudioBuffer.dma + (PAGE_SIZE <<1)* audio_ch;
		dmaB = dev->m_AudioBuffer.dma + (PAGE_SIZE <<1)* audio_ch + PAGE_SIZE;

			reg_writel( DMA_CH8_CONFIG_P + audio_ch*2, dmaP );
		/// Audio B = P+1
			reg_writel( DMA_CH8_CONFIG_B + audio_ch*2, dmaB );

		/// printk("---Audio DMA 0x19-0x29 config --- CH%02d   write dmaP 0X%p  dmaB 0X%p  read 0X%x 0X%x   \n",
		///	    audio_ch, dmaP, dmaB, reg_readl(DMA_CH8_CONFIG_P + audio_ch*2), reg_readl(DMA_CH8_CONFIG_B + audio_ch*2) );

	}

    ///printk(KERN_INFO " Mmio %s: CFG[0x78]  cpu 0x%x    dma 0x%x \n", dev->name,  (unsigned int)dev->m_Page0.cpu, dev->m_Page0.dma );

    regDW = reg_readl( (DMA_PAGE_TABLE0_ADDR) );
    printk(KERN_INFO "DMA %s: DMA_PAGE_TABLE0_ADDR  0x%x    \n",  dev->name, regDW );
    regDW = reg_readl( (DMA_PAGE_TABLE1_ADDR) );
    printk(KERN_INFO "DMA %s: DMA_PAGE_TABLE1_ADDR  0x%x    \n",  dev->name, regDW );

    reg_writel((DMA_PAGE_TABLE0_ADDR), dev->m_Page0.dma);						//P DMA page table
    reg_writel((DMA_PAGE_TABLE1_ADDR), dev->m_Page0.dma + (PAGE_SIZE <<1));		//B DMA page table


    regDW = reg_readl( (DMA_PAGE_TABLE0_ADDR) );
    printk(KERN_INFO "DMA %s: DMA_PAGE_TABLE0_ADDR  0x%x    \n",  dev->name, regDW );
    regDW = reg_readl( (DMA_PAGE_TABLE1_ADDR) );
    printk(KERN_INFO "DMA %s: DMA_PAGE_TABLE1_ADDR  0x%x    \n",  dev->name, regDW );

	/*
    regDW = tw_readl( (DMA_PAGE_TABLE0_ADDR) );
    printk(KERN_INFO "DMA %s: tw DMA_PAGE_TABLE0_ADDR  0x%x    \n",  dev->name, regDW );
    regDW = tw_readl( (DMA_PAGE_TABLE1_ADDR) );
    printk(KERN_INFO "DMA %s: tw DMA_PAGE_TABLE1_ADDR  0x%x    \n",  dev->name, regDW );
	*/

	reg_writel(AVSRST,0x3F);        // u32
    regDW = reg_readl(AVSRST);
	printk(KERN_INFO "DMA %s: tw AVSRST _u8 %x :: 0x%x    \n",  dev->name, (AVSRST<<2), regDW );

	reg_writel(DMA_CMD, 0);          // u32
    regDW = reg_readl( DMA_CMD );
	printk(KERN_INFO "DMA %s: tw DMA_CMD _u8 %x :: 0x%x    \n",  dev->name, (DMA_CMD<<2), regDW );

	reg_writel(DMA_CHANNEL_ENABLE,0);
    regDW = reg_readl( DMA_CHANNEL_ENABLE );
	printk(KERN_INFO "DMA %s: tw DMA_CHANNEL_ENABLE %x :: 0x%x    \n",  dev->name, DMA_CHANNEL_ENABLE, regDW );

    regDW = reg_readl( DMA_CHANNEL_ENABLE );
	printk(KERN_INFO "DMA %s: tw DMA_CHANNEL_ENABLE %x :: 0x%x    \n",  dev->name, DMA_CHANNEL_ENABLE, regDW );
	reg_writel(DMA_CHANNEL_TIMEOUT, 0x180c8F88);   //  860 a00  0x140c8560  0x1F0c8b08     0xF00F00   140c8E08   0x140c8D08
	//reg_writel(DMA_CHANNEL_TIMEOUT, 0x3EFF0FF0);   // no liutest longer time out
    regDW = reg_readl( DMA_CHANNEL_TIMEOUT );
	printk(KERN_INFO "DMA %s: tw DMA_CHANNEL_TIMEOUT %x :: 0x%x    \n",  dev->name, DMA_CHANNEL_TIMEOUT, regDW );


	reg_writel(DMA_INT_REF, 0x38000);   ///   2a000 2b000 2c000  3932e     0x3032e
    regDW = reg_readl( DMA_INT_REF );
	printk(KERN_INFO "DMA %s: tw DMA_INT_REF %x :: 0x%x    \n",  dev->name, DMA_INT_REF, regDW );


	reg_writel(DMA_CONFIG, 0x00FF0004);
    regDW = reg_readl( DMA_CONFIG );
	printk(KERN_INFO "DMA %s: tw DMA_CONFIG %x :: 0x%x    \n",  dev->name, DMA_CONFIG, regDW );


	regDW = (0xFF << 16) | (VIDEO_GEN_PATTERNS <<8) | VIDEO_GEN;

	printk(KERN_INFO " set tw68 VIDEO_CTRL2 %x :: 0x%x    \n",  VIDEO_CTRL2, regDW );

	reg_writel(VIDEO_CTRL2, regDW);
	regDW = reg_readl( VIDEO_CTRL2 );
	printk(KERN_INFO "DMA %s: tw DMA_CONFIG %x :: 0x%x    \n",  dev->name, VIDEO_CTRL2, regDW );

	//VDelay
	regDW = 0x014;

	reg_writel(VDELAY0, regDW);
	reg_writel(VDELAY1, regDW);
	reg_writel(VDELAY2, regDW);
	reg_writel(VDELAY3, regDW);
	// +4 decoder   0x100
	//   6869
	reg_writel(VDELAY0 +0x100, regDW);
	reg_writel(VDELAY1 +0x100, regDW);
	reg_writel(VDELAY2 +0x100, regDW);
	reg_writel(VDELAY3 +0x100, regDW);
	

	//Show Blue background if no signal
	regDW = 0xe7;
	reg_writel(MISC_CONTROL2, regDW);

	// 6869
	reg_writel(MISC_CONTROL2 +0x100, regDW);

    regDW = reg_readl( VDELAY0 );
	printk(KERN_INFO " read tw68 VDELAY0 %x :: 0x%x    \n",  VDELAY0, regDW );

    regDW = reg_readl( MISC_CONTROL2 );
	printk(KERN_INFO " read tw68 MISC_CONTROL2 %x :: 0x%x    \n",  MISC_CONTROL2, regDW );



	//////////////////////////////////////
	// 2864 #1/External, Muxed-656
	//Reset 2864s

	val1  = reg_readl(CSR_REG);
	val1 &= 0x7FFF;
	reg_writel(CSR_REG, val1);
	/// printk("2864 init CSR_REG 0x%x]=  I2C 2864   val1:0X%x  %x\n", CSR_REG,  val1 );

	mdelay(100);
	val1 |= 0x8002;  // Pull out from reset and enable I2S
	reg_writel(CSR_REG, val1);
	/// printk("2864 init CSR_REG 0x%x]=  I2C 2864   val1:0X%x  %x\n", CSR_REG,  val1 );



	addr = CLKOCTRL | 0x100;
	///val0 = DeviceRead2864(dev, addr);
	val1 = 0x40 | ( VIDEO_IN_MODE | (VIDEO_IN_MODE<<2) ) ; // Out enable
	///DeviceWrite2864(dev, addr, (unsigned char)val1);
	///val2=DeviceRead2864(dev, addr);
	/// printk("2864[CLKOCTRL 0x%x]=  I2C 2864  val0:0X%x   val1:0X%x   val2:0X%x\n", CLKOCTRL, val0, val1, val2 );


	addr = NOVID | 0x100;
	val1 = 0x73;					// CHID with 656 Sync code, 656 output even no video input
	///val0 = DeviceRead2864(dev,addr);
	///DeviceWrite2864(dev,addr,val1);
	///val2 = DeviceRead2864(dev,addr);


	// device data structure initialization
	TW68_video_init1(dev);

    // decoder parameter setup
	TW68_video_init2(dev);   // set TV param



	dev->video_DMA_1st_started =0;  // initial value for skipping startup DMA error
	dev->err_times =0;  // DMA error counter

	dev->TCN = 16;

	for (k=0; k<8; k++)
	{
		dev->videoDMA_run[k]=0;;
	}

	return 0;
}




/* shutdown */
static int TW68_hwfini(struct TW68_dev *dev)

{
	dprintk("hwfini\n");

	return 0;
}



static int vdev_init8 (struct TW68_dev *dev, struct video_device *template,  char *type)
{
    struct video_device* vfdev[9];     /// QF 0 + 8

    int k=1;
	int err0;

    for ( k =1; k<9; k++ )     // dev0  QF muxer  dev 1 ~ 4
    {
        vfdev[k] = video_device_alloc();

        //vfdev[k] = vfd;

        if (NULL == vfdev[k])
        {
            printk(KERN_WARNING "Null  vfdev %d **** \n\n", k);
            return k;

        }
        *(vfdev[k]) = *template;

        vfdev[k]->v4l2_dev  = &dev->v4l2_dev;
        vfdev[k]->release = video_device_release;
        vfdev[k]->debug   = video_debug;
        snprintf(vfdev[k]->name, sizeof(vfdev[k]->name), "%s %s (%s22)",
             dev->name, type, TW68_boards[dev->board].name);

        dev->video_device[k] = vfdev[k];


		printk(KERN_INFO "*****%d*****video DEVICE NAME : %s   vfdev[%d] 0x%p  tvnorm::0x%p \n", k, dev->video_device[k]->name, k, vfdev[k], dev->tvnorm);   //  vfdev[k]->name,


		err0 = video_register_device(dev->video_device[k], VFL_TYPE_GRABBER, video_nr[dev->nr]);
		dev->vfd_DMA_num[k] = vfdev[k]->num;

		printk(KERN_INFO "*****%d*****video DEVICE NAME : %s   minor %d   DMA %d  err0 %d \n", k, vfdev[k]->name, dev->video_device[k]->minor, dev->vfd_DMA_num[k], err0);
	}

    printk(KERN_INFO "%s Video DEVICE NAME : %s  \n", __func__, dev->video_device[1]->name);

    return k;
}


static void TW68_unregister_video(struct TW68_dev *dev)
{
    int k;
    for ( k =1; k<9; k++ )   /// 0 + 4
    {

	if (dev->video_device[k]) 
		if (-1 != dev->video_device[k]->minor)
		{
			video_unregister_device(dev->video_device[k]);

			printk(KERN_INFO "video_unregister_device(dev->video_dev %d  \n", k );
		}
	
    }
}


static int __devinit TW68_initdev(struct pci_dev *pci_dev,
				     const struct pci_device_id *pci_id)
{
	struct TW68_dev *dev;
	int err, err0;

	printk(KERN_INFO "PCI register init called\n");

	 if (TW68_devcount == TW68_MAXBOARDS)
		return -ENOMEM;

	dev = kzalloc(sizeof(*dev),GFP_KERNEL);
	if (NULL == dev)
		return -ENOMEM;

	err = v4l2_device_register(&pci_dev->dev, &dev->v4l2_dev);
	if (err)
		goto fail0;

	/* pci init */
	dev->pci = pci_dev;
	if (pci_enable_device(pci_dev)) {
		err = -EIO;
		goto fail1;
	}

	dev->nr = TW68_devcount;
	sprintf(dev->name,"TW%x[%d]",pci_dev->device,dev->nr);

	printk(KERN_INFO " %s TW68_devcount: %d \n", dev->name, TW68_devcount );

	/* pci quirks */
	if (pci_pci_problems) {
		if (pci_pci_problems & PCIPCI_TRITON)
			printk(KERN_INFO "%s: quirk: PCIPCI_TRITON\n", dev->name);
		if (pci_pci_problems & PCIPCI_NATOMA)
			printk(KERN_INFO "%s: quirk: PCIPCI_NATOMA\n", dev->name);
		if (pci_pci_problems & PCIPCI_VIAETBF)
			printk(KERN_INFO "%s: quirk: PCIPCI_VIAETBF\n", dev->name);
		if (pci_pci_problems & PCIPCI_VSFX)
			printk(KERN_INFO "%s: quirk: PCIPCI_VSFX\n",dev->name);
#ifdef PCIPCI_ALIMAGIK
		if (pci_pci_problems & PCIPCI_ALIMAGIK) {
			printk(KERN_INFO "%s: quirk: PCIPCI_ALIMAGIK -- latency fixup\n",
			       dev->name);
			latency = 0x0A;
		}
#endif
		if (pci_pci_problems & (PCIPCI_FAIL|PCIAGP_FAIL)) {
			printk(KERN_INFO "%s: quirk: this driver and your "
					"chipset may not work together"
					" in overlay mode.\n",dev->name);
			if (!TW68_no_overlay) {
				printk(KERN_INFO "%s: quirk: overlay "
						"mode will be disabled.\n",
						dev->name);
				TW68_no_overlay = 1;
			} else {
				printk(KERN_INFO "%s: quirk: overlay "
						"mode will be forced. Use this"
						" option at your own risk.\n",
						dev->name);
			}
		}
	}

/*
	if (UNSET != latency) {
		printk(KERN_INFO "%s: setting pci latency timer to %d\n",
		       dev->name,latency);
		pci_write_config_byte(pci_dev, PCI_LATENCY_TIMER, latency);
	}
*/

	/* print pci info */
	pci_read_config_byte(pci_dev, PCI_CLASS_REVISION, &dev->pci_rev);
	pci_read_config_byte(pci_dev, PCI_LATENCY_TIMER,  &dev->pci_lat);

	printk(KERN_INFO "%s: found at %s, rev: %d, irq: %d, "
	       "latency: %d, mmio: 0x%llx\n", dev->name,
	       pci_name(pci_dev), dev->pci_rev, pci_dev->irq,
	       dev->pci_lat,(unsigned long long)pci_resource_start(pci_dev,0));

	pci_set_master(pci_dev);

	if (!pci_dma_supported(pci_dev, DMA_BIT_MASK(32))) {
		printk("%s: Oops: no 32bit PCI DMA ???\n",dev->name);
		err = -EIO;
		goto fail1;
	}
	else
        printk("%s: Hi: 32bit PCI DMA supported \n",dev->name);


    dev->board = 1;
    printk(KERN_INFO "%s: subsystem: %04x:%04x, board: %s [card=%d,%s]\n",
		dev->name,pci_dev->subsystem_vendor,
		pci_dev->subsystem_device,TW68_boards[dev->board].name,
		dev->board, dev->autodetected ?
		"autodetected" : "insmod option");


	/* get mmio */
	if (!request_mem_region(pci_resource_start(pci_dev,0),
				pci_resource_len(pci_dev,0),
				dev->name)) {
		err = -EBUSY;
		printk(KERN_ERR "%s: can't get MMIO memory @ 0x%llx\n",
		       dev->name,(unsigned long long)pci_resource_start(pci_dev,0));
		goto fail1;
	}

	// no cache
	dev->lmmio = ioremap_nocache(pci_resource_start(pci_dev, 0),  pci_resource_len(pci_dev, 0));

	dev->bmmio = (__u8 __iomem *)dev->lmmio;

	if (NULL == dev->lmmio) {
		err = -EIO;
		printk(KERN_ERR "%s: can't ioremap() MMIO memory\n",
		       dev->name);
		goto fail2;
	}

   	//printk(KERN_INFO "	TW6869 PCI_BAR0 mapped registers: phy: 0x%X   dev->lmmio 0X%x  dev->bmmio 0X%x   length: %x \n",
        //  pci_resource_start(pci_dev, 0), (unsigned int)dev->lmmio, (unsigned int)dev->bmmio, (unsigned int)pci_resource_len(pci_dev,0) );

	/* initialize hardware #1 */
	TW68_hwinit1(dev);
	///InitExternal2864(dev);
	/* get irq */
    printk( "TW68_initdev   %s: request IRQ %d\n",     dev->name,pci_dev->irq);

	 err = request_irq(pci_dev->irq, TW68_irq,  IRQF_SHARED , dev->name, dev);  ///   IRQF_SHARED | IRQF_DISABLED

	 // liutest add tasklet
	 tasklet_init(&dev->bf_tasklet, videobuf_tasklet, (unsigned long)dev);
	 tasklet_init(&dev->resync_tasklet, resync_tasklet, (unsigned long)dev);

	 if (err < 0) {
		printk(KERN_ERR "%s: can't get IRQ %d\n",
		       dev->name,pci_dev->irq);
		goto fail3;
	 }

	v4l2_prio_init(&dev->prio);

	printk(KERN_ERR "Adding  TW686v_devlist %p\n", &TW686v_devlist);
	list_add_tail(&dev->devlist, &TW686v_devlist);
	////add current TW68_dev device structure node

	/* register v4l devices */
	if (TW68_no_overlay > 0)
		printk(KERN_INFO "%s: Overlay support disabled.\n", dev->name);
    else
       	printk(KERN_INFO "%s: Overlay supported %d .\n", dev->name, TW68_no_overlay);

   err0 = vdev_init8(dev, &TW68_video_template,"video");    

	if (err0 < 0) {
		printk(KERN_INFO "%s: can't register video device\n",
		       dev->name);
		goto fail4;
	}

    TW68_devcount++;
    printk(KERN_INFO "%s: registered PCI device %d [v4l2]:%d  err: |%d| \n",
	       dev->name, TW68_devcount, dev->video_device[1]->num, err0);

	return 0;


 fail4:
	TW68_unregister_video(dev);
	 free_irq(pci_dev->irq, dev);
 fail3:
	TW68_hwfini(dev);
	iounmap(dev->lmmio);
 fail2:
	release_mem_region(pci_resource_start(pci_dev,0),
			   pci_resource_len(pci_dev,0));
 fail1:
	v4l2_device_unregister(&dev->v4l2_dev);
 fail0:
	kfree(dev);
	return err;
}

static void __devexit TW68_finidev(struct pci_dev *pci_dev)
{

	int m, n, k = 0;
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pci_dev);
	struct TW68_dev *dev = container_of(v4l2_dev, struct TW68_dev, v4l2_dev);

	printk(KERN_INFO "%s: Starting unregister video device %d\n",
	       dev->name, dev->video_device[1]->num);


	printk(KERN_INFO " /* shutdown hardware */ dev 0x%p \n" , dev);

	/* shutdown hardware */

	 TW68_hwfini(dev);

	/* shutdown subsystems */

	/* unregister */
	mutex_lock(&TW68_devlist_lock);
	list_del(&dev->devlist);
	mutex_unlock(&TW68_devlist_lock);
	TW68_devcount--;

	printk(KERN_INFO " list_del(&dev->devlist) \n ");


	/* the DMA sound modules should be unloaded before reaching
	   this, but just in case they are still present... */
	//if (dev->dmasound.priv_data != NULL) {
	// free_irq(pci_dev->irq, &dev->dmasound);
	//	dev->dmasound.priv_data = NULL;
	//}

	del_timer(&dev->delay_resync);

	/* release resources */
	/// remove IRQ
	free_irq(pci_dev->irq, dev);     /////////  0420
	iounmap(dev->lmmio);
	release_mem_region(pci_resource_start(pci_dev,0),
			   pci_resource_len(pci_dev,0));

    TW68_pgtable_free(dev->pci, &dev->m_Page0);


	for (n =0; n<8; n++)
	for (m =0; m<4; m++)
	{
		pci_free_consistent(dev->pci, 800*300*2, dev->BDbuf[n][m].cpu, dev->BDbuf[n][m].dma_addr);
	}

    TW68_pgtable_free(dev->pci, &dev->m_AudioBuffer);

	for ( k =0; k<8; k++)
	{
		dma_field_free(&dev->Field_P[k]);
		dma_field_free(&dev->Field_B[k]);
	}

	 TW68_unregister_video(dev);
	 printk(KERN_INFO " TW68_unregister_video(dev); \n ");

	//v4l2_device_unregister(&dev->v4l2_dev);
	printk(KERN_INFO " unregistered v4l2_dev device \n");
	/* free memory */
	kfree(dev);
}



/* ----------------------------------------------------------- */

static struct pci_driver TW68_pci_driver = {
	.name     = "TW6868",
	.id_table = TW68_pci_tbl,
	.probe    = TW68_initdev,
	.remove   = __devexit_p(TW68_finidev),

//#ifdef CONFIG_PM
//	.suspend  = TW6868_suspend,
//	.resume   = TW6868_resume
//#endif
};
/* ----------------------------------------------------------- */

static int TW68_init(void)
{
	INIT_LIST_HEAD(&TW686v_devlist);
	printk(KERN_INFO "TW6868_: v4l2 driver version %d.%d.%d loaded\n", 2, 0, 1);

	return pci_register_driver(&TW68_pci_driver);
}

static void TW68_fini(void)
{
	pci_unregister_driver(&TW68_pci_driver);
}

module_init(TW68_init);
module_exit(TW68_fini);


/* ----------------------------------------------------------- */
/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */



