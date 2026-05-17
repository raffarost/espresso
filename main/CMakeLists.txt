idf_component_register(
    SRCS ./main.c ./control.c ./rainmaker.c ./tasks.c ./nvs.c
         ./pid.c ./math.c
    INCLUDE_DIRS "." "../components/max6675")
