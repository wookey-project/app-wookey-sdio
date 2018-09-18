/**
 * @file main.c
 *
 * \brief Main of dummy
 *
 */

#include "api/syscall.h"
#include "api/print.h"
#include "libsdio.h"
#include "libsd.h"
#include "ipc_proto.h"

/*
 * We use the local -fno-stack-protector flag for main because
 * the stack protection has not been initialized yet.
 *
 * We use _main and not main to permit the usage of exactly *one* arg
 * without compiler complain. argc/argv is not a goot idea in term
 * of size and calculation in a microcontroler
 */
uint8_t sdio_rd_buf[4000];
uint8_t sdio_wr_buf[4000];

int _main(uint32_t task_id)
{
    e_syscall_ret ret;
    char *wellcome_msg = "hello, I'm sdio";
    struct sync_command ipc_sync_cmd;
    uint8_t id_crypto;
    uint8_t id;
    dma_shm_t dmashm_rd;
    dma_shm_t dmashm_wr;

    printf("%s, my id is %x\n", wellcome_msg, task_id);

    /* Early init phase of drivers/libs */
    if(sd_early_init())
      printf("SDIO KO !!!!! \n");

    ret = sys_init(INIT_GETTASKID, "crypto", &id_crypto);
    printf("crypto is task %x !\n", id_crypto);

    /*********************************************
     * Declaring DMA Shared Memory with Crypto
     *********************************************/
    dmashm_rd.target = id_crypto;
    dmashm_rd.source = task_id;
    dmashm_rd.address = (physaddr_t)sdio_rd_buf;
    dmashm_rd.size = 4000;
    /* Crypto DMA will read from this buffer */
    dmashm_rd.mode = DMA_SHM_ACCESS_RD;

    dmashm_wr.target = id_crypto;
    dmashm_wr.source = task_id;
    dmashm_wr.address = (physaddr_t)sdio_wr_buf;
    dmashm_wr.size = 4000;
    /* Crypto DMA will write into this buffer */
    dmashm_wr.mode = DMA_SHM_ACCESS_WR;

    printf("Declaring DMA_SHM for SDIO read flow\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_rd);
    printf("sys_init returns %s !\n", strerror(ret));

    printf("Declaring DMA_SHM for SDIO write flow\n");
    ret = sys_init(INIT_DMA_SHM, &dmashm_wr);
    printf("sys_init returns %s !\n", strerror(ret));

    /*******************************************
     * End of init
     *******************************************/

    printf("set init as done\n");
    ret = sys_init(INIT_DONE);
    printf("sys_init returns %s !\n", strerror(ret));

    /*******************************************
     * let's syncrhonize with other tasks
     *******************************************/
    logsize_t size = 2;

    printf("sending end_of_init synchronization to crypto\n");
    ipc_sync_cmd.magic = MAGIC_TASK_STATE_CMD;
    ipc_sync_cmd.state = SYNC_READY;

    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_crypto, size, (char*)&ipc_sync_cmd);
    } while (ret != SYS_E_DONE);

    /* Now wait for Acknowledge from Smart */
    id = id_crypto;

    do {
        ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    } while (ret != SYS_E_DONE);
    if (   ipc_sync_cmd.magic == MAGIC_TASK_STATE_RESP
        && ipc_sync_cmd.state == SYNC_ACKNOWLEDGE) {
        printf("crypto has acknowledge end_of_init, continuing\n");
    }

    /*******************************************
     * Starting end_of_cryp synchronization
     *******************************************/

    printf("waiting end_of_cryp syncrhonization from crypto\n");

    id = id_crypto;
    size = 2;

    do {
        ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    } while (ret != SYS_E_DONE);

    if (   ipc_sync_cmd.magic == MAGIC_TASK_STATE_CMD
        && ipc_sync_cmd.state == SYNC_READY) {
        printf("crypto module is ready\n");
    }

    /* init phase of drivers/libs */
    sd_init();

    ipc_sync_cmd.magic = MAGIC_TASK_STATE_RESP;
    ipc_sync_cmd.state = SYNC_READY;

    size = 2;
    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_crypto, size, (char*)&ipc_sync_cmd);
    } while (ret != SYS_E_DONE);

    /*******************************************
     * Starting USB listener
     *******************************************/



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
    while (1) {
      sys_yield();
    }

    return 0;
}

