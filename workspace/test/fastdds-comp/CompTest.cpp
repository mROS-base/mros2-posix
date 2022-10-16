/*
 *
 * Author: Andreas Wüstenberg (andreas.wuestenberg@rwth-aachen.de), Clemens Boennen (clemens.boennen@rwth-aachen.de)
 */

#include <array>
#include <vector>
#include <chrono>
#include <iostream>


#include "rtps/rtps.h"
#include "rtps/entities/Domain.h"
#include "Sender.h"
#include "Receiver.h"

#include <algorithm>
#include "cmsis_os.h"
#include "lwip.h"

#define REAL_TIME 0

#if REAL_TIME
#include <sys/mman.h>

#endif

static bool isResponder = false;
static const uint32_t numSamples = 10;

void startProgram();
int startPrioritizedProgram();
#include "netif_posix_add.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main(int argc,char* argv[])
{
  if ((argc != 3) && (argc != 4)) {
    printf("Usage: %s <ipaddr> <netmask> [send]\n", argv[0]);
    return 1;
  }
  netif_posix_add(argv[1], argv[2]);
  osKernelStart();
  MX_LWIP_Init();

  if(argc == 4) {
    std::cout << "Starting sender unit\n";
    isResponder = false;
  } else if(argc == 3) {
    std::cout << "Starting receiver unit\n";
    isResponder = true;
  } else {
    std::cout << std::string(R"("Please execute with argument "publisher" or "subscriber")") << std::endl;
    return -1;
  }

  //rtps::init();

#if REAL_TIME
  std::cout << "Running with real-time priority.\n";
  return startPrioritizedProgram();
#else
  std::cout << "Running with normal priority.\n";
  startProgram();
#endif
  return 0;
}

void startProgram()
{

  if(isResponder) {
    rtps::tests::Receiver receiver(128);
    receiver.run();
    while(true);
  } else {
    rtps::tests::Sender sender(numSamples);
    sender.run();
    while(true);
  }
}

void* thread_func(void* /*args*/)
{
  startProgram();
  return NULL;
}

#if REAL_TIME
int startPrioritizedProgram()
{
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  int ret;

  // Lock memory
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
    printf("mlockall failed: %m\n");
    exit(-2);
  }

  // Initialize pthread attributes (default values)
  ret = pthread_attr_init(&attr);
  if (ret) {
    printf("init pthread attributes failed\n");
    goto out;
  }

  // Set a specific stack size
  ret = pthread_attr_setstacksize(&attr, 1e+7);
  if (ret) {
    printf("pthread setstacksize failed\n");
    goto out;
  }

  // Set scheduler policy and priority of pthread
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret) {
    printf("pthread setschedpolicy failed\n");
    goto out;
  }
  param.sched_priority = 98;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret) {
    printf("pthread setschedparam failed\n");
    goto out;
  }

  // Use scheduling parameters of attr
  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret) {
    printf("pthread setinheritsched failed\n");
    goto out;
  }

  /* Create a pthread with specified attributes */
  ret = pthread_create(&thread, &attr, thread_func, nullptr);
  if (ret) {
    printf("create pthread failed\n");
    goto out;
  }
  /* Join the thread and wait until it is done */
  ret = pthread_join(thread, NULL);
  if (ret) {
    printf("join pthread failed: %m\n");
  }

out:
  return ret;

}
#endif
