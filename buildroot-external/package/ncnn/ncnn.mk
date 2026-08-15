################################################################################
# ncnn
################################################################################

NCNN_VERSION = 20260526
NCNN_SITE = https://github.com/Tencent/ncnn/archive/refs/tags
NCNN_SOURCE = $(NCNN_VERSION).tar.gz
NCNN_INSTALL_STAGING = YES
NCNN_INSTALL_TARGET = YES
NCNN_CONF_OPTS = \
	-DNCNN_SHARED_LIB=ON \
	-DNCNN_VULKAN=OFF \
	-DNCNN_OPENMP=OFF \
	-DNCNN_THREADS=ON \
	-DNCNN_BUILD_EXAMPLES=OFF \
	-DNCNN_BUILD_TOOLS=OFF \
	-DNCNN_BUILD_BENCHMARK=OFF \
	-DNCNN_DISABLE_RTTI=ON \
	-DNCNN_DISABLE_EXCEPTION=ON

$(eval $(cmake-package))
