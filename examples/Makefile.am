INCLUDE = -I${includedir} -I../include

AM_CFLAGS = -Wall -g -O2 $(INCLUDE)

bin_PROGRAMS = rpmsg_char_simple rpmsg_char_benchmark

# rpmsg_char_simple program
rpmsg_char_simple_SOURCES = rpmsg_char_simple.c
rpmsg_char_simple_CPPFLAGS = $(AM_CFLAGS)
rpmsg_char_simple_LDADD = -lpthread ../src/libti_rpmsg_char.la

# rpmsg_char_benchmark program
rpmsg_char_benchmark_SOURCES = rpmsg_char_benchmark.c
rpmsg_char_benchmark_CPPFLAGS = $(AM_CFLAGS)
rpmsg_char_benchmark_LDADD = -lpthread ../src/libti_rpmsg_char.la
