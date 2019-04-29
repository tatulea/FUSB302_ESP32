#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_PRIV_INCLUDEDIRS := . core core/modules core/vdm
								
COMPONENT_SRCDIRS := . core core/modules core/vdm

CFLAGS += -DPLATFORM_NONE \
			-DFSC_HAVE_SRC \
			-DFSC_HAVE_DRP \
			-DFSC_HAVE_SNK