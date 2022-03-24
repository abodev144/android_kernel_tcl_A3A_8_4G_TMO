#
# Makefile for building the SELinux module as part of the kernel tree.
#

obj-$(CONFIG_SECURITY_SELINUX) := selinux.o

selinux-y := avc.o hooks.o selinuxfs.o netlink.o nlmsgtab.o netif.o \
	     netnode.o netport.o exports.o \
	     ss/ebitmap.o ss/hashtab.o ss/symtab.o ss/sidtab.o ss/avtab.o \
	     ss/policydb.o ss/services.o ss/conditional.o ss/mls.o ss/status.o

selinux-$(CONFIG_SECURITY_NETWORK_XFRM) += xfrm.o

selinux-$(CONFIG_NETLABEL) += netlabel.o

ccflags-y := -Isecurity/selinux -Isecurity/selinux/include

$(addprefix $(obj)/,$(selinux-y)): $(obj)/flask.h

quiet_cmd_flask = GEN     $(obj)/flask.h $(obj)/av_permissions.h
      cmd_flask = scripts/selinux/genheaders/genheaders $(obj)/flask.h $(obj)/av_permissions.h

targets += flask.h av_permissions.h
$(obj)/flask.h: $(src)/include/classmap.h FORCE
	$(call if_changed,flask)

#[FEATURE]-Mod-BEGIN by SCDTABLET.(qli3@tcl.com), 2018/03/12, 6052796, add USER2ENG feature
ifeq ($(ENABLE_USER2ENG), true)
KBUILD_CFLAGS += -DENABLE_USER2ENG
endif
#[FEATURE]-Mod-END by SCDTABLET.(qli3@tcl.com),2018/03/12 ,6052796, add USER2ENG feature
