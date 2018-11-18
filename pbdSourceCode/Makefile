# -----------------------------------------------------------------------------
# CMake project wrapper Makefile ----------------------------------------------
# -----------------------------------------------------------------------------

SHELL := /bin/bash
RM    := rm -rf
MKDIR := mkdir -p
BUILDROOT := Build
BUILDDEFAULT := Release
BUILDDIR = $(BUILDROOT)/$(BUILDDEFAULT)/
ROOTDIR := $(CURDIR)
BUILDTYPES := Release Debug MinSizeRel Coverage RelWithDebInfo Quick
OTHERGOALS := $(filter-out distclean $(BUILDTYPES),$(MAKECMDGOALS))

# Call make from build directory
.PHONY: all
all: ./$(BUILDROOT)/Release/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Call make from build directory
.PHONY: Release
Release: BUILDDIR = $(BUILDROOT)/Release/
Release: ./$(BUILDROOT)/Release/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Call make from build directory
.PHONY: MinSizeRel
MinSizeRel: BUILDDIR = $(BUILDROOT)/MinSizeRel/
MinSizeRel: ./$(BUILDROOT)/MinSizeRel/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Call make from build directory
.PHONY: RelWithDebInfo
RelWithDebInfo: BUILDDIR = $(BUILDROOT)/RelWithDebInfo/
RelWithDebInfo: ./$(BUILDROOT)/RelWithDebInfo/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Call make from build directory
.PHONY: Debug
Debug: BUILDDIR = $(BUILDROOT)/Debug/
Debug: ./$(BUILDROOT)/Debug/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Call make from build directory
.PHONY: Coverage
Coverage: BUILDDIR = $(BUILDROOT)/Coverage/
Coverage: ./$(BUILDROOT)/Coverage/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Call make from build directory
.PHONY: Quick
Quick: BUILDDIR = $(BUILDROOT)/Quick/
Quick: ./$(BUILDROOT)/Quick/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)

# Make build directory and call cmake
./$(BUILDROOT)/%/Makefile:
	@ ($(MKDIR) $(BUILDROOT)/$* > /dev/null)
	@ (cd $(BUILDROOT)/$* > /dev/null 2>&1 && cmake -DCMAKE_BUILD_TYPE=$* $(ROOTDIR))

# remove everything
distclean:
	@  ($(MKDIR) $(BUILDDIR) > /dev/null)
	@  (cd $(BUILDDIR) > /dev/null 2>&1 && cmake $(ROOTDIR) > /dev/null 2>&1)
	@- $(MAKE) --silent -C $(BUILDDIR) clean || true
	@- $(RM) ./$(BUILDDIR)/Makefile
	@- $(RM) ./$(BUILDDIR)/CMake*
	@- $(RM) ./$(BUILDDIR)/cmake.*
	@- $(RM) ./$(BUILDDIR)/*.cmake
	@- $(RM) ./$(BUILDDIR)/*.txt

# for all other goals pass them down to make in the build directory
ifeq ($(filter distclean $(BUILDTYPES),$(MAKECMDGOALS)),)

$(OTHERGOALS): ./$(BUILDDIR)/Makefile
	@ $(MAKE) -s -C $(BUILDDIR) $(MAKEFLAGS) $(OTHERGOALS)
else

$(OTHERGOALS): 
	@true
endif
