INCLUDE = -I${includedir}

AM_CFLAGS = -Wall -g -O2 $(INCLUDE)

bin_PROGRAMS = rpmsg_char_simple

# rpmsg_char_simple program
rpmsg_char_simple_SOURCES = rpmsg_char_simple.c
rpmsg_char_simple_CPPFLAGS = $(AM_CFLAGS)
rpmsg_char_simple_LDADD = -lpthread -lti_rpmsg_char
rpmsg_char_simple_LDFLAGS = -L${libdir}
