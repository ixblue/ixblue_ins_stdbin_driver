# - Try to find libiXblue_stdbin_decoder include dirs and libraries
#
# Usage of this module as follows:
#
#     find_package(iXblue_stdbin_decoder)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  iXblue_stdbin_decoder_ROOT_DIR              Set this variable to the root installation of
#                                               libiXblue_stdbin_decoder if the module has problems finding the
#                                               proper installation path.
#
# Variables defined by this module:
#
#  iXblue_stdbin_decoder_FOUND                System has libiXblue_stdbin_decoder, include and library dirs found
#  iXblue_stdbin_decoder_INCLUDE_DIR          The libiXblue_stdbin_decoder include directories.
#  iXblue_stdbin_decoder_LIBRARY              The libiXblue_stdbin_decoder library (possibly includes a thread
#                                              library e.g. required by pf_ring's libiXblue_stdbin_decoder)

find_path(iXblue_stdbin_decoder_ROOT_DIR
    HINTS /usr/local
    NAMES include/iXblue_stdbin_decoder
)

find_path(iXblue_stdbin_decoder_INCLUDE_DIR
    NAMES iXblue_stdbin_decoder/stdbin_decoder.h
    HINTS ${iXblue_stdbin_decoder_ROOT_DIR}/include
)

find_library(iXblue_stdbin_decoder_LIBRARIES
    NAMES iXblue_stdbin_decoder
    HINTS ${iXblue_stdbin_decoder_ROOT_DIR}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(iXblue_stdbin_decoder DEFAULT_MSG
    iXblue_stdbin_decoder_LIBRARIES
    iXblue_stdbin_decoder_INCLUDE_DIR
)

mark_as_advanced(
    iXblue_stdbin_decoder_ROOT_DIR
    iXblue_stdbin_decoder_INCLUDE_DIR
    iXblue_stdbin_decoder_LIBRARIES
)

