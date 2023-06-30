lib_LTLIBRARIES = libti_rpmsg_char.la

include_HEADERS = \
		$(top_srcdir)/include/ti_rpmsg_char.h \
		$(top_srcdir)/include/rproc_id.h

libti_rpmsg_char_la_SOURCES = rpmsg_char.c soc.c utils.c

libti_rpmsg_char_la_CFLAGS = -I$(top_srcdir)/include

libti_rpmsg_char_la_LDFLAGS = -version-number 0:6:2 -no-undefined
