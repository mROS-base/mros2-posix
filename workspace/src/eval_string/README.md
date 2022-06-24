# eval_string

## Build

- Edit workspace/include/rtps/config.h and configure `rtps::Config::IP_ADDRESS`
- Run build script:
```shell
# at repository root
bash build.sh eval_string all
```

## Run

```shell
# echo node
./cmake-build/mros2-posix echo <netmask>
# pub node
./cmake-build/mros2-posix pub <platform> <length of a string> <netmask>
# sub node
./cmake-build/mros2-posix sub <platform> <length of a string> <netmask>
```
