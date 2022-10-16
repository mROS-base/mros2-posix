#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

static in_addr_t get_ipaddr(const char* addrp)
{
  return inet_addr(addrp);
}

#include "cmsis_os.h"
#include "netif_posix_add.h"

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/igmp.h"
#include "lwip/init.h"
#include "lwip.h"

#define MY_PORT	(7400 + (250 * 0) + 1)

typedef enum {
  TestCmd_Recv,
  TestCmd_Send
} TestCmdType;
static char* addr_to_inet_addr(uint32_t ipaddr);
static void sender(udp_pcb* pcb);
static void my_udp_recv_fn(void* arg, struct udp_pcb* pcb, struct pbuf* p, const ip_addr_t* addr, u16_t port);
static void receiver(udp_pcb* pcb);
static in_addr_t multicast_ipaddr;
static in_addr_t local_ipaddr;

int main(int argc, const char* argv[])
{
  if ((argc != 3) && (argc != 4)) {
    printf("Usage: %s <ipaddr> <netmask> [send]\n", argv[0]);
    return 1;
  }
  netif_posix_add(argv[1], argv[2]);

  TestCmdType cmd = TestCmd_Recv;
  if (argc != 3) {
    cmd = TestCmd_Send;
  }
  printf("TestCmd=%d port=%d\n", cmd, MY_PORT);

  osKernelStart();
  MX_LWIP_Init();
  udp_pcb* pcb = udp_new();
  if (pcb == NULL) {
    printf("ERROR: pcb_new returns NULL\n");
    return 1;
  }
  local_ipaddr = get_ipaddr(argv[1]);
  err_t err = udp_bind(pcb, (const ip_addr_t*)&local_ipaddr, MY_PORT);
  if (err != ERR_OK) {
    printf("ERROR: udp_bind returns %d\n", err);
    return 1;
  }
  multicast_ipaddr = get_ipaddr("239.255.0.1");
  err = igmp_joingroup((const ip4_addr_t*)&local_ipaddr, (const ip4_addr_t*)&multicast_ipaddr);
  if (err != ERR_OK) {
    printf("ERROR: igmp_joingroup returns %d\n", err);
    return 1;
  }

  if (cmd == TestCmd_Recv) {
    receiver(pcb);
  } else {
    sender(pcb);
  }


  udp_remove(pcb);
  return 0;
}

static void sender(udp_pcb* pcb)
{
  struct pbuf buffer;
  const char* msg = "Hello World!!";
  buffer.payload = (void*)msg;
  buffer.len = strlen(msg) + 1;
  buffer.tot_len = strlen(msg) + 1;
  buffer.next = NULL;

  while (true) {
    err_t err = udp_sendto(pcb, &buffer, (const ip4_addr_t*)&multicast_ipaddr, MY_PORT);
    CMSIS_IMPL_INFO("send data:port=%d err=%d", pcb->local_port, err);
    osDelay(1000);
  }
  return;
}
static char* addr_to_inet_addr(uint32_t ipaddr)
{
  static char buffer[1024];
  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "%d.%d.%d.%d",
           (ipaddr & (0xFF << 0U)) >> 0,
           (ipaddr & (0xFF << 8U)) >> 8,
           (ipaddr & (0xFF << 16U)) >> 16,
           (ipaddr & (0xFF << 24U)) >> 24
          );
  return buffer;
}

static void my_udp_recv_fn(void* arg, struct udp_pcb* pcb, struct pbuf* p,
                           const ip_addr_t* addr, u16_t port)
{
  CMSIS_IMPL_INFO("recv data:addr=%s port=%d payload=%s", addr_to_inet_addr(addr->addr), port, (char*)p->payload);

  pbuf_free(p);
  return;
}

static void receiver(udp_pcb* pcb)
{
  const char* recv_arg = "received data:";
  udp_recv(pcb, my_udp_recv_fn, (void*)recv_arg);
  while (true) {
    osDelay(1000);
  }
  return;
}


#include "mros2.h"
#include "mros2_user_config.h"
#include "std_msgs/msg/string.hpp"

int mros2_get_submsg_count(void)
{
  return 10;
}