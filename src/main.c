/**
 * @file main.c
 *
 * \brief Main of dummy
 *
 */

#include "autoconf.h"
#include "libc/syscall.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libsdio.h"
#include "libsd.h"
#include "wookey_ipc.h"
#include "generated/led1.h"


/*
 * We use the local -fno-stack-protector flag for main because
 * the stack protection has not been initialized yet.
 *
 * We use _main and not main to permit the usage of exactly *one* arg
 * without compiler complain. argc/argv is not a goot idea in term
 * of size and calculation in a microcontroler
 */
#define SDIO_DEBUG 0
#define SDIO_BUF_SIZE 16384

/* NOTE: alignment due to DMA */
__attribute__((aligned(4))) uint8_t sdio_buf[SDIO_BUF_SIZE] = { 0 };

extern volatile uint8_t SD_ejection_occured;


static inline void led_on(void)
{
    /* toggle led ON */
    sys_cfg(CFG_GPIO_SET, (uint8_t)((led1_dev_infos.gpios[LED1].port << 4) + led1_dev_infos.gpios[LED1].pin), 1);
}


static inline void led_off(void)
{
    /* toggle led OFF */
    sys_cfg(CFG_GPIO_SET, (uint8_t)((led1_dev_infos.gpios[LED1].port << 4) + led1_dev_infos.gpios[LED1].pin), 0);
}


void SDIO_asks_reset(uint8_t id_crypto)
{
  struct sync_command ipc_sync_cmd;

  ipc_sync_cmd.magic=0x86;//MAGIC_STORAGE_EJECTED;
  ipc_sync_cmd.state=SYNC_WAIT;

  sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct sync_command), (char*)&ipc_sync_cmd);
}


int _main(uint32_t task_id)
{
    e_syscall_ret ret;
    char *wellcome_msg = "hello, I'm sdio";
    struct sync_command ipc_sync_cmd;
    struct sync_command_data ipc_sync_cmd_data;
    uint8_t id_crypto;
    uint8_t id;
    dma_shm_t dmashm_rd;
    dma_shm_t dmashm_wr;
    int led_desc;

    printf("%s, my id is %x\n", wellcome_msg, task_id);

    /* Early init phase of drivers/libs */
    if(sd_early_init()){
        printf("SDIO KO !!!!! \n");
    }

    ret = sys_init(INIT_GETTASKID, "crypto", &id_crypto);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }

    printf("crypto is task %x !\n", id_crypto);

    /*********************************************
     * Declaring DMA Shared Memory with Crypto
     *********************************************/
    dmashm_rd.target = id_crypto;
    dmashm_rd.source = task_id;
    dmashm_rd.address = (physaddr_t)sdio_buf;
    dmashm_rd.size = SDIO_BUF_SIZE;
    /* Crypto DMA will read from this buffer */
    dmashm_rd.mode = DMA_SHM_ACCESS_RD;

    dmashm_wr.target = id_crypto;
    dmashm_wr.source = task_id;
    dmashm_wr.address = (physaddr_t)sdio_buf;
    dmashm_wr.size = SDIO_BUF_SIZE;
    /* Crypto DMA will write into this buffer */
    dmashm_wr.mode = DMA_SHM_ACCESS_WR;

    printf("Declaring DMA_SHM for SDIO read flow\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_rd);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }
    printf("sys_init returns %s !\n", strerror(ret));

    printf("Declaring DMA_SHM for SDIO write flow\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_wr);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }
    printf("sys_init returns %s !\n", strerror(ret));

#if CONFIG_WOOKEY
    /*********************************************
     * Declaring SDIO read/write access LED
     ********************************************/
    printf("Declaring SDIO LED device\n");
    device_t dev;
    memset(&dev, 0, sizeof(device_t));
    strncpy(dev.name, "sdio_led", sizeof("sdio_led"));
    dev.gpio_num = 1;
    dev.gpios[0].mask = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_SPEED;
    dev.gpios[0].kref.port = led1_dev_infos.gpios[LED1].port;
    dev.gpios[0].kref.pin = led1_dev_infos.gpios[LED1].pin;
    dev.gpios[0].pupd = GPIO_NOPULL;
    dev.gpios[0].mode = GPIO_PIN_OUTPUT_MODE;
    dev.gpios[0].speed = GPIO_PIN_HIGH_SPEED;

    ret = sys_init(INIT_DEVACCESS, &dev, &led_desc);
    if (ret != SYS_E_DONE) {
        printf("Error while declaring LED GPIO device: %d\n", ret);
        goto error;
    }
#endif

    /*******************************************
     * End of init
     *******************************************/

    printf("set init as done\n");
    ret = sys_init(INIT_DONE);
    printf("sys_init returns %s !\n", strerror(ret));

#if CONFIG_WOOKEY
    led_off();
#endif

    /*******************************************
     * let's syncrhonize with other tasks
     *******************************************/
    logsize_t size = sizeof(struct sync_command);

    printf("sending end_of_init synchronization to crypto\n");
    ipc_sync_cmd.magic = MAGIC_TASK_STATE_CMD;
    ipc_sync_cmd.state = SYNC_READY;

    ret = sys_ipc(IPC_SEND_SYNC, id_crypto, size, (char*)&ipc_sync_cmd);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }

    /* Now wait for Acknowledge from Smart */
    id = id_crypto;

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }

    if (   (ipc_sync_cmd.magic == MAGIC_TASK_STATE_RESP)
            && (ipc_sync_cmd.state == SYNC_ACKNOWLEDGE)) {
        printf("crypto has acknowledge end_of_init, continuing\n");
    }

    /*******************************************
     * Starting end_of_cryp synchronization
     *******************************************/

    printf("waiting end_of_cryp syncrhonization from crypto\n");

    id = id_crypto;
    size = sizeof(struct sync_command);

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }

    if (   (ipc_sync_cmd.magic == MAGIC_TASK_STATE_CMD)
            && (ipc_sync_cmd.state == SYNC_READY)) {
        printf("crypto module is ready\n");
    }

    /* init phase of drivers/libs */
    if(SD_ejection_occured){
        SDIO_asks_reset(id_crypto);
    }
    sd_init();

    /************************************************
     * Sending crypto end_of_service_init
     ***********************************************/

    ipc_sync_cmd.magic = MAGIC_TASK_STATE_RESP;
    ipc_sync_cmd.state = SYNC_READY;
    size = sizeof(struct sync_command);

    ret = sys_ipc(IPC_SEND_SYNC, id_crypto, size, (char*)&ipc_sync_cmd);
    if (ret != SYS_E_DONE) {
        printf("sending end of services init to crypto: Oops ! ret = %d\n", ret);
        goto error;
    } else {
        printf("sending end of services init to crypto ok\n");
    }

    /* Waiting for crypto acknowledge */
    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if (ret != SYS_E_DONE) {
        printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
        goto error;
    }

    if (   (ipc_sync_cmd.magic == MAGIC_TASK_STATE_RESP)
        && (ipc_sync_cmd.state == SYNC_ACKNOWLEDGE)) {
        printf("crypto has acknowledge sync ready, continuing\n");
    } else {
        printf("Error ! IPC desynchro !\n");
    }

    /*******************************************
     * Sharing DMA SHM address and size with crypto
     *******************************************/

    ipc_sync_cmd_data.magic = MAGIC_DMA_SHM_INFO_CMD;
    ipc_sync_cmd_data.state = SYNC_READY;
    ipc_sync_cmd_data.data_size = 2;
    ipc_sync_cmd_data.data.u32[0] = (uint32_t)sdio_buf;
    ipc_sync_cmd_data.data.u32[1] = SDIO_BUF_SIZE;

    printf("informing crypto about DMA SHM...\n");
    ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct sync_command_data), (char*)&ipc_sync_cmd_data);
    if (ret != SYS_E_DONE) {
        printf("sys_ipc(IPC_SEND_SYNC, id_crypto) failed! Exiting...\n");
        goto error;
    }

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if(ret != SYS_E_DONE){
        printf("sys_ipc(IPC_RECV_SYNC) failed! Exiting...\n");
        goto error;
    }
    if (   (ipc_sync_cmd.magic == MAGIC_DMA_SHM_INFO_RESP)
        && (ipc_sync_cmd.state == SYNC_ACKNOWLEDGE)) {
        printf("crypto has acknowledge DMA SHM, continuing\n");
    } else {
        printf("Error ! IPC desynchro !\n");
        goto error;
    }

    printf("Crypto informed.\n");

    /*******************************************
     * Main read/write loop
     *   SDIO is waiting for READ/WRITE command
     *   from IPC interface
     *******************************************/

    printf("SDIO main loop starting\n");

    /*
     * Main waiting loopt. The task main thread is awoken by any external
     * event such as ISR or IPC.
     */
    struct dataplane_command dataplane_command_wr;
    struct dataplane_command dataplane_command_ack = { MAGIC_DATA_WR_DMA_ACK, 0, 0, 0 };
    t_ipc_command ipc_mainloop_cmd;
    memset(&ipc_mainloop_cmd, 0, sizeof(t_ipc_command));

    while (1) {
        uint8_t id = id_crypto;
        uint8_t sd_ret;
        logsize_t size = sizeof(t_ipc_command);

        ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_mainloop_cmd);

        if (ret != SYS_E_DONE) {
            printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
            goto error;
        }

#if 0
        if((ret != SYS_E_DONE) && (!SD_ejection_occured )) {
            continue;
        }
#endif

        switch (ipc_mainloop_cmd.magic) {

            case MAGIC_STORAGE_SCSI_BLOCK_SIZE_CMD:
            {
                ipc_sync_cmd_data = ipc_mainloop_cmd.sync_cmd_data;
                ipc_sync_cmd_data.magic = MAGIC_STORAGE_SCSI_BLOCK_SIZE_RESP;
                ipc_sync_cmd_data.state = SYNC_DONE;
                ipc_sync_cmd_data.data_size = 1;
                ipc_sync_cmd_data.data.u32[0] = sd_get_block_size();
                /***************************************************
                 * SDIO/USB block size synchronization
                 **************************************************/
                /* now that SDIO has returned, let's return to USB */
                ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct sync_command_data), (char*)&ipc_sync_cmd_data);
                if(ret != SYS_E_DONE){
                    printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
                    goto error;
                }
                break;
            }

            case MAGIC_STORAGE_SCSI_BLOCK_NUM_CMD:
            {
                /***************************************************
                 * SDIO/USB block number synchronization
                 **************************************************/
                ipc_sync_cmd_data = ipc_mainloop_cmd.sync_cmd_data;
                ipc_sync_cmd_data.magic = MAGIC_STORAGE_SCSI_BLOCK_NUM_RESP;
                ipc_sync_cmd_data.state = SYNC_DONE;
                ipc_sync_cmd_data.data_size = 2;
                ipc_sync_cmd_data.data.u32[0] = sd_get_block_size();
                ipc_sync_cmd_data.data.u32[1] = sd_get_capacity();
                /***************************************************
                 * SDIO/USB block size synchronization
                 **************************************************/
                /* now that SDIO has returned, let's return to USB */
                ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct sync_command_data), (char*)&ipc_sync_cmd_data);
                if(ret != SYS_E_DONE){
                    printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
                    goto error;
                }
                break;
            }

            case MAGIC_DATA_WR_DMA_REQ:
            {
#if CONFIG_WOOKEY
                led_on();
#endif
                dataplane_command_wr = ipc_mainloop_cmd.dataplane_cmd;
#if SDIO_DEBUG
                printf("!!!!!!!!!!! received DMA write command to SDIO: @:%x size: %d\n",
                        dataplane_command_wr.sector_address, dataplane_command_wr.num_sectors);

                printf("!!!!!!!!!! WRITE dumping SDIO buf @:%x size: %d\n", dataplane_command_wr.sector_address, dataplane_command_wr.num_sectors);
#endif
                // write request.... let's write then...
                sd_ret = sd_write((uint32_t*)sdio_buf, dataplane_command_wr.sector_address, 512*dataplane_command_wr.num_sectors);

                dataplane_command_ack.magic = MAGIC_DATA_WR_DMA_ACK;
                if(sd_error != SD_SUCCESS || sd_ret != SD_SUCCESS) {
                    printf("sd_write() failure : R1 register %x status reg %x\n", saver1, savestatus);
                    dataplane_command_ack.state = SYNC_FAILURE;
                } else {
                    dataplane_command_ack.state = SYNC_ACKNOWLEDGE;
                }

#if CONFIG_WOOKEY
                led_off();
#endif
                ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct dataplane_command), (const char*)&dataplane_command_ack);
                if(ret != SYS_E_DONE){
                    printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
                    goto error;
                }
                break;
            }

            case MAGIC_DATA_RD_DMA_REQ:
            {
#if CONFIG_WOOKEY
                led_on();
#endif
                dataplane_command_wr = ipc_mainloop_cmd.dataplane_cmd;
#if SDIO_DEBUG
                printf("received DMA read command to SDIO: @[sector] :%x @[bytes]: %x size: %d\n",
                        dataplane_command_wr.sector_address,
                        dataplane_command_wr.sector_address * 512,
                        dataplane_command_wr.num_sectors);
#endif
                // read request.... let's read then...

                sd_ret = sd_read((uint32_t*)sdio_buf, dataplane_command_wr.sector_address, 512*dataplane_command_wr.num_sectors);

                dataplane_command_ack.magic = MAGIC_DATA_RD_DMA_ACK;
                if(sd_error != SD_SUCCESS || sd_ret != SD_SUCCESS) {
                    printf("sd_read() failure : R1 register %x status reg %x\n", saver1, savestatus);
                    dataplane_command_ack.state = SYNC_FAILURE;
                } else {
                    dataplane_command_ack.state = SYNC_ACKNOWLEDGE;
                }

#if CONFIG_WOOKEY
                led_off();
#endif
                ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct dataplane_command), (const char*)&dataplane_command_ack);
                if(ret != SYS_E_DONE){
                    printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
                    goto error;
                }
                break;
            }

            default:
            {
                printf("received invalid command from CRYPTO (magic: %d\n", ipc_mainloop_cmd.magic);
                ipc_mainloop_cmd.magic = MAGIC_INVALID;
                ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(t_ipc_command), (const char*)&ipc_mainloop_cmd);
                if(ret != SYS_E_DONE){
                    printf("%s:%d Oops ! ret = %d\n", __func__, __LINE__, ret);
                    goto error;
                }
                break;
            }
        }

        /*
         *  The Card is not present in the connector
         *  We indicate this to crypto that will forward the information
         *  to afferent apps
         */
        if (SD_ejection_occured) {
            SDIO_asks_reset(id_crypto);
        }
    }

 error:
    /* Should never be reached except in case of error!!! */
    return 1;
}

