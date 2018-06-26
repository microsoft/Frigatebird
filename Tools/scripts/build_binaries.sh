#!/bin/bash
# script to build the latest binaries for each vehicle type, ready to upload
# Andrew Tridgell, March 2013

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.binaries
echo $TMDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date
git checkout -f master
githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H:%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries
BASEDIR=$PWD

error_count=0

. config.mk

board_branch() {
    board="$1"
    case $board in
        apm1|apm2)
            echo "-AVR"
            ;;
        *)
            echo ""
            ;;
    esac
}

# add board specific options
board_options() {
    board="$1"
    case $board in
        bebop)
            # bebop needs a static build
            echo "--static"
            ;;
        *)
            echo ""
            ;;
    esac
}

waf() {
    if [ -x ./waf ]; then
        ./waf "$@"
    else
        ./modules/waf/waf-light "$@"
    fi
}

# checkout the right version of the tree
checkout() {
    vehicle="$1"
    ctag="$2"
    cboard="$3"
    cframe="$4"
    echo "Trying checkout $vehicle $ctag $cboard $cframe"
    git stash
    if [ "$ctag" = "latest" ]; then
        vtag="master"
    else
        vtag="$vehicle-$ctag"
    fi

    # try frame specific tag
    if [ -n "$cframe" ]; then
        vtag2="$vtag-$cframe"

        git checkout -f "$vtag2" && {
            echo "Using frame specific tag $vtag2"
            [ -f $BASEDIR/.gitmodules ] && git submodule update --recursive -f
            git log -1
            return 0
        }
    fi

    # try board type specific branch extension
    vtag2="$vtag"$(board_branch $cboard)

    git checkout -f "$vtag2" && {
        echo "Using board specific tag $vtag2"
        [ -f $BASEDIR/.gitmodules ] && git submodule update --recursive -f
        git log -1
        return 0
    }

    git checkout -f "$vtag" && {
        echo "Using generic tag $vtag"
        [ -f $BASEDIR/.gitmodules ] && git submodule update --recursive -f
        git log -1
        return 0
    }

    echo "Failed to find tag for $vehicle $ctag $cboard $cframe"
    return 1
}

# check if we should skip this build because we don't
# support the board in this release
skip_board() {
    b="$1"
    if grep -q "$b" ../mk/targets.mk; then
        return 1
    fi
    echo "Skipping unsupported board $b"
    return 0
}

# check if we should skip this build because we don't
# support the board in this release
skip_board_waf() {
    b="$1"
    if grep -q "$b" $BASEDIR/Tools/ardupilotwaf/boards.py; then
        return 1
    fi
    echo "Skipping unsupported board $b"
    return 0
}

skip_frame() {
    sboard=$1
    sframe=$2
    if [ "$sboard" = "bebop" -o "$sboard" = "aerofc-v1" ]; then
        if [ "$sframe" != "quad" -a "$sframe" != "none" ]; then
            return 0
        fi
    fi
    return 1
}

# check if we should skip this build because we have already
# built this version
skip_build() {
    [ "$FORCE_BUILD" = "1" ] && return 1
    tag="$1"
    ddir="$2"
    bname=$(basename $ddir)
    ldir=$(dirname $(dirname $(dirname $ddir)))/$tag/$bname
    [ -f $BASEDIR/.gitmodules ] || {
        echo "Skipping build without submodules"
        return 0
    }
    [ -d "$ldir" ] || {
        echo "$ldir doesn't exist - building"
        return 1
    }
    oldversion=$(cat "$ldir/git-version.txt" | head -1)
    newversion=$(git log -1 | head -1)
    [ "$oldversion" = "$newversion" ] && {
        echo "Skipping build - version match $newversion"
        return 0
    }
    echo "$ldir needs rebuild"
    return 1
}

addfwversion() {
    destdir="$1"
    src="$2"
    git log -1 > "$destdir/git-version.txt"
    versionfile="$src/version.h"
    [ -f $versionfile ] && {
        shopt -s nullglob
        version=$(grep 'define.THISFIRMWARE' $versionfile 2> /dev/null | cut -d'"' -f2)
        echo >> "$destdir/git-version.txt"
        echo "APMVERSION: $version" >> "$destdir/git-version.txt"
        python $BASEDIR/Tools/PrintVersion.py $src >"$destdir/firmware-version.txt"
    }    
}

# copy the built firmware to the right directory
copyit() {
    file="$1"
    dir="$2"
    tag="$3"
    src="${4:-.}"
    bname=$(basename $dir)
    tdir=$(dirname $(dirname $(dirname $dir)))/$tag/$bname
    if [ "$tag" = "latest" ]; then
        mkdir -p "$dir"
        /bin/cp "$file" "$dir"
        addfwversion "$dir" "$src"
    fi
    echo "Copying $file to $tdir"
    mkdir -p "$tdir"
    addfwversion "$tdir" "$src"

    rsync "$file" "$tdir"
}

board_extension() {
    board="$1"
    case $board in
        apm1|apm2)
            echo "hex"
            ;;
        *)
            echo "elf"
            ;;
    esac
}

# build plane binaries
build_arduplane() {
    tag="$1"
    echo "Building ArduPlane $tag binaries from $(pwd)"
    pushd ArduPlane
    for b in apm1 apm2; do
        checkout ArduPlane $tag $b "" || {
            echo "Failed checkout of ArduPlane $b $tag"
            error_count=$((error_count+1))
            continue
        }
        skip_board $b && continue
        echo "Building ArduPlane $b binaries"
        ddir=$binaries/Plane/$hdate/$b
        skip_build $tag $ddir && continue
        make clean || continue
        make $b -j4 || {
            echo "Failed build of ArduPlane $b $tag"
            error_count=$((error_count+1))
            continue
        }
        extension=$(board_extension $b)
        copyit $BUILDROOT/ArduPlane.$extension $ddir $tag
        touch $binaries/Plane/$tag
    done
    popd
    for b in erlebrain2 navio navio2 pxf pxfmini disco; do
        checkout ArduPlane $tag $b "" || {
            echo "Failed checkout of ArduPlane $b $tag"
            error_count=$((error_count+1))
            continue
        }
        skip_board_waf $b && continue
        echo "Building ArduPlane $b binaries"
        ddir=$binaries/Plane/$hdate/$b
        skip_build $tag $ddir && continue
        waf configure --board $b --out $BUILDROOT clean plane || {
            echo "Failed build of ArduPlane $b $tag"
            error_count=$((error_count+1))
            continue
        }
        copyit $BUILDROOT/$b/bin/arduplane $ddir $tag "ArduPlane"
        touch $binaries/Plane/$tag
    done
    pushd ArduPlane
    echo "Building ArduPlane PX4 binaries"
    ddir=$binaries/Plane/$hdate/PX4
    checkout ArduPlane $tag PX4 "" || {
        echo "Failed checkout of ArduPlane PX4 $tag"
        error_count=$((error_count+1))
        checkout ArduPlane "latest" "" ""
        popd
        return
    }
    skip_build $tag $ddir || {
        for v in v1 v2 v3 v4; do
            make px4-clean
            make px4-$v -j2 || {
                echo "Failed build of ArduPlane PX4 $tag for $v"
                error_count=$((error_count+1))
                checkout ArduPlane "latest" "" ""
                popd
                return
            }
        done
        copyit ArduPlane-v1.px4 $ddir $tag &&
        copyit ArduPlane-v2.px4 $ddir $tag &&
        test ! -f ArduPlane-v3.px4 || copyit ArduPlane-v3.px4 $ddir $tag &&
        test ! -f ArduPlane-v4.px4 || copyit ArduPlane-v4.px4 $ddir $tag
        if [ "$tag" = "latest" ]; then
            copyit px4io-v1.bin $binaries/PX4IO/$hdate/PX4IO $tag
            copyit px4io-v1.elf $binaries/PX4IO/$hdate/PX4IO $tag
            copyit px4io-v2.bin $binaries/PX4IO/$hdate/PX4IO $tag
            copyit px4io-v2.elf $binaries/PX4IO/$hdate/PX4IO $tag
        fi
    }
    checkout ArduPlane "latest" "" ""
    popd
}

[ -f .gitmodules ] && {
    git submodule init
    git submodule update --recursive -f
}

echo "Copying over a replacement ardupilotmega.xml file to odules/mavlink/message_definitions/v1.0/"
rsync --checksum ardupilotmega.xml modules/mavlink/message_definitions/v1.0/

export BUILDROOT="$TMPDIR/binaries.build"
rm -rf $BUILDROOT

# make sure PX4 is rebuilt from scratch
for d in ArduPlane; do
         pushd $d
         make px4-clean || exit 1
         popd
done

for build in stable beta latest; do
    build_arduplane $build
done

rm -rf $TMPDIR

if ./Tools/scripts/generate-manifest.py $binaries http://firmware.ardupilot.org >$binaries/manifest.json.new; then
    echo "Manifest generation succeeded"
    # provide a pre-compressed manifest.  For reference, a 7M manifest
    # "gzip -9"s to 300k in 1 second, "xz -e"s to 80k in 26 seconds
    gzip -9 <$binaries/manifest.json.new >$binaries/manifest.json.gz.new
    mv $binaries/manifest.json.new $binaries/manifest.json
    mv $binaries/manifest.json.gz.new $binaries/manifest.json.gz
else
    echo "Manifest generation failed"
fi

exit $error_count
