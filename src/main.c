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
#define SDIO_DEBUG 0
#define SDIO_BUF_SIZE 16384
uint8_t sdio_buf[SDIO_BUF_SIZE] = { 0 };

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
    logsize_t size = sizeof(struct sync_command);

    printf("sending end_of_init synchronization to crypto\n");
    ipc_sync_cmd.magic = MAGIC_TASK_STATE_CMD;
    ipc_sync_cmd.state = SYNC_READY;

    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_crypto, size, (char*)&ipc_sync_cmd);
    } while (ret == SYS_E_BUSY);

    /* Now wait for Acknowledge from Smart */
    id = id_crypto;

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);
    if (   ipc_sync_cmd.magic == MAGIC_TASK_STATE_RESP
        && ipc_sync_cmd.state == SYNC_ACKNOWLEDGE) {
        printf("crypto has acknowledge end_of_init, continuing\n");
    }

    /*******************************************
     * Starting end_of_cryp synchronization
     *******************************************/

    printf("waiting end_of_cryp syncrhonization from crypto\n");

    id = id_crypto;
    size = sizeof(struct sync_command);

    ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_sync_cmd);

    if (   ipc_sync_cmd.magic == MAGIC_TASK_STATE_CMD
        && ipc_sync_cmd.state == SYNC_READY) {
        printf("crypto module is ready\n");
    }

    /* init phase of drivers/libs */
    sd_init();

    ipc_sync_cmd.magic = MAGIC_TASK_STATE_RESP;
    ipc_sync_cmd.state = SYNC_READY;
    size = sizeof(struct sync_command);
    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_crypto, size, (char*)&ipc_sync_cmd);
    } while (ret == SYS_E_BUSY);
    // take some time to finish all sync ipc...
    sys_sleep(2000, SLEEP_MODE_INTERRUPTIBLE);

    /*******************************************
     * Sharing DMA SHM address and size with crypto
     *******************************************/
    struct dmashm_info {
        uint32_t addr;
        uint16_t size;
    };

    struct dmashm_info dmashm_info;

    dmashm_info.addr = (uint32_t)sdio_buf;
    dmashm_info.size = SDIO_BUF_SIZE;

    printf("informing crypto about DMA SHM...\n");
    do {
      ret = sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct dmashm_info), (char*)&dmashm_info);
    } while (ret == SYS_E_BUSY);
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
      struct dataplane_command dataplane_command_ack = { DATA_WR_DMA_ACK, 0, 0 };
      t_ipc_command ipc_mainloop_cmd = { 0 };

      while (1) {
          uint8_t id = id_crypto;
          logsize_t size = sizeof(t_ipc_command);

          ret = sys_ipc(IPC_RECV_SYNC, &id, &size, (char*)&ipc_mainloop_cmd);

          if (ret != SYS_E_DONE) {
              continue;
          }

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
                    sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct sync_command_data), (char*)&ipc_sync_cmd_data);

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
                    sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct sync_command_data), (char*)&ipc_sync_cmd_data);

                    break;
                }



              case DATA_WR_DMA_REQ:
                  {
                    dataplane_command_wr = ipc_mainloop_cmd.dataplane_cmd;
#if SDIO_DEBUG
                    printf("!!!!!!!!!!! received DMA write command to SDIO: @:%x size: %d\n",
                            dataplane_command_wr.sector_address, dataplane_command_wr.num_sectors);

                    printf("!!!!!!!!!! WRITE dumping SDIO buf @:%x size: %d\n", dataplane_command_wr.sector_address, dataplane_command_wr.num_sectors);
#endif
                    // write request.... let's write then...
                    sd_write((uint32_t*)sdio_buf, dataplane_command_wr.sector_address, 512*dataplane_command_wr.num_sectors);

                    dataplane_command_ack.magic = DATA_WR_DMA_ACK;

                    sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct dataplane_command), (const char*)&dataplane_command_ack);
                    break;

                  }

              case DATA_RD_DMA_REQ:
                  {
                    dataplane_command_wr = ipc_mainloop_cmd.dataplane_cmd;
#if SDIO_DEBUG
                    printf("received DMA read command to SDIO: @[sector] :%x @[bytes]: %x size: %d\n",
                            dataplane_command_wr.sector_address,
                            dataplane_command_wr.sector_address * 512,
                            dataplane_command_wr.num_sectors);
#endif
                    // read request.... let's read then...

                    sd_read((uint32_t*)sdio_buf, dataplane_command_wr.sector_address, 512*dataplane_command_wr.num_sectors);

                    dataplane_command_ack.magic = DATA_RD_DMA_ACK;

                    sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(struct dataplane_command), (const char*)&dataplane_command_ack);
                    break;

                  }


              default:
                  {
                      printf("received invalid command from CRYPTO (magic: %d\n", ipc_mainloop_cmd.magic);
                      ipc_mainloop_cmd.magic = MAGIC_INVALID;
                      sys_ipc(IPC_SEND_SYNC, id_crypto, sizeof(t_ipc_command), (const char*)&ipc_mainloop_cmd);
                      break;

                  }
          }
      }

    return 0;
}

