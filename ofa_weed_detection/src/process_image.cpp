#include <cstdio>
#include <k4a/k4a.h>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  uint32_t count = k4a_device_get_installed_count();

  printf("test: %u", count);


  return 0;
}
