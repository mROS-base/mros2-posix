#include <cstdio>
#include <cstring>

int pub_main(int argc, char *argv[]);
int sub_main(int argc, char *argv[]);
int echo_main(int argc, char *argv[]);

extern "C" int mros2_get_submsg_count(void) { return 10; }

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("usage: %s (pub|sub|echo) <args>...\n", argv[0]);
        return 1;
    } else if (std::strncmp(argv[1], "pub", 3) == 0) {
        printf("Run in pub mode\n");
        return pub_main(argc, argv);
    } else if (std::strncmp(argv[1], "sub", 3) == 0) {
        printf("Run in sub mode\n");
        return sub_main(argc, argv);
    } else {
        printf("Run in echo mode\n");
        return echo_main(argc, argv);
    }
}
