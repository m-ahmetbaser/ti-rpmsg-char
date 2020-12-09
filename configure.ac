#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])

# Initilize autoconf
AC_INIT([ti-rpmsg-char], [0.3.0], [s-anna@ti.com])

# Check common src files to ensure integrity of the package
AC_CONFIG_SRCDIR([src/rpmsg_char.c])
AC_CONFIG_HEADERS([config.h])

# Set auxiliary build tools directories
AC_CONFIG_AUX_DIR([build-aux])

# Initialize automake
AM_INIT_AUTOMAKE([foreign])

# Disabling maintainer mode to prevent rebuild of configure script
AM_MAINTAINER_MODE([disable])

# Initialize libtool
AC_PROG_LIBTOOL

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL

# Checks for libraries.

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([fcntl.h stddef.h stdint.h stdlib.h string.h sys/ioctl.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_TYPE_SIZE_T
AC_TYPE_UINT32_T

# Checks for library functions.
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset realpath strerror strtol])

# Export CFLAGS and LDFLAGS to Makefile(s)
AC_SUBST([AM_CFLAGS])
AC_SUBST([AM_LDFLAGS])


# Generate Makefile(s)
AC_CONFIG_FILES([Makefile
                 src/Makefile
                 src/ti_rpmsg_char.pc
                 examples/Makefile])
AC_OUTPUT

echo \
"-----------------------------------------------------------

Thank you for installing the Linux ${PACKAGE_NAME} package

Installation directory prefix: '${prefix}'.
Compilation command: '${CC} ${CFLAGS} ${CPPFLAGS}'

Now type 'make; sudo make install' to generate the program
and install it to your system. sudo usage is optional and
depends on the permissions of the prefix folder being used
or installation.

------------------------------------------------------------"
