#!/bin/bash
set -e  # exit on first error

DEPS_DIR="/tmp/wave_dependencies"

main()
{
    install_geographiclib
}

install_geographiclib()
{
    GEOGRAPHICLIB_VERSION="1.49"
    GEOGRAPHICLIB_URL="https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-$GEOGRAPHICLIB_VERSION.tar.gz"
    GEOGRAPHICLIB_DIR="GeographicLib-$GEOGRAPHICLIB_VERSION"

    if (ldconfig -p | grep -q libGeographic.so.17 ); then
        echo "GeographicLib version $GEOGRAPHICLIB_VERSION is already installed."
    else
        echo "Installing GeographicLib version $GEOGRAPHICLIB_VERSION ..."
        mkdir -p "$DEPS_DIR"
        cd "$DEPS_DIR"
        wget "$GEOGRAPHICLIB_URL"
        tar -xf "GeographicLib-$GEOGRAPHICLIB_VERSION.tar.gz"
        rm -rf "GeographicLib-$GEOGRAPHICLIB_VERSION.tar.gz"

        cd "$GEOGRAPHICLIB_DIR"
        mkdir -p BUILD
        cd BUILD
        cmake ..

        make_with_progress -j$(nproc)
        make test
        sudo make install > /dev/null
    fi
}

main