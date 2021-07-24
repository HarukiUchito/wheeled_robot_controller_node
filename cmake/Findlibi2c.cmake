# インクルードディレクトリのパスをgmp.hを頼りに検索する
find_path(LIBI2C_INCLUDE_DIR /home/xoke/gitRepos/libi2c/include)
# ライブラリへのパスをライブラリ名を元に検索する
find_library(LIBI2C_LIBRARY
  NAMES
    i2c
  PATHS
    /home/xoke/gitRepos
  PATH_SUFFIXES
    lib
  )
# advancedモードでない限り変数の存在を隠す
mark_as_advanced(LIBI2C_INCLUDE_DIR LIBI2C_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBI2C
  REQUIRED_VARS
    LIBI2C_INCLUDE_DIR
    LIBI2C_LIBRARY
  )

if(LIBI2C_FOUND AND NOT TARGET LIBI2C::LIBI2C)
  # GMP::GMPというターゲット名でGMPライブラリを定義
  # UNKNOWN = STATIC/SHAREDかはまだ不明
  # IMPORTED = このプロジェクトに属さないターゲット
  add_library(LIBI2C::LIBI2C UNKNOWN IMPORTED)
  set_target_properties(LIBI2C::LIBI2C PROPERTIES
    # C言語、C++なら"CXX"とする
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${LIBI2C_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBI2C_INCLUDE_DIR}"
    )
endif()