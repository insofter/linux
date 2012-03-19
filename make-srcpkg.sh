#!/bin/sh

print_usage()
{
>&2 cat <<EOF

Usage: ${program_name} [-o|--output-dir OUTPUT_DIR]
  [-h|--help] [-v|--version]

Builds a source package for linux-stable project. The output directory
can be specified by -o|--output-dir parameter. If the parameter is not
provided then current directory is used. The script must be executed
from source top directory.

  -o|--output-dir output directory
  -h|--help       show this information
  -v|--version    show version information

EOF
}

print_version()
{
>&2 cat <<EOF

${program_name} ${version}
Copyright (c) 2011-2012 Tomasz Rozensztrauch

EOF
}

info() 
{
  echo "${program_name}: $1" >&2
}

error() 
{
  echo "${program_name}: Error! $1" >&2
  if [ "$2" != "noexit" ]; then
    exit 1;
  fi
}

program_name=`basename "$0"`
version=`git describe --dirty | sed -e 's/^v\([^+]*\)+\(.*\)$/\1-\2/'`
tag=`git describe --abbrev=0 | sed -e 's/^[^+]*+\(.*\)$/-\1/'`

output_dir=`pwd`

options=`getopt -o o:hv --long output-dir:,help,version -- "$@"`
test $? -eq 0 || error "Parsing parameters failed"
eval set -- "$options"
while true ; do
  case "$1" in
    -o|--output-dir) output_dir=`eval cd "$2" && pwd`;
       test $? -eq 0 || error "Invalid output directory specified"; shift 2 ;;
    -h|--help) print_usage; exit 0 ;;
    -v|--version) print_version; exit 0 ;;
    --) shift; break ;;
    *) error "Parsing parameters failed at '$1'" ;;
  esac
done

test "x$1" = "x" || error "Parsing parameters failed at '$1'"

temp_dir=`mktemp -d`
test $? -eq 0 || error "Creating temporary directory '${temp_dir}' failed"
export_dir="${temp_dir}/linux-${version}"
mkdir -p "${export_dir}"
test $? -eq 0 || error "Creating '${export_dir}' directory failed"

info "Preparing source directory tree..."
rsync -a --exclude=.git* --exclude=/*sh --exclude=/CMakeLists.txt ./ "${export_dir}"
test $? -eq 0 || error "Copying package files failed"

info "Updating 'scripts/setlocalversion'..."
cat scripts/setlocalversion | sed -e 's/^local_tag=.*$/local_tag="'"${tag}"'"/' \
  -e '/sed:package {/,/sed:package }/ d' > ${export_dir}/scripts/setlocalversion
chmod a+x ${export_dir}/scripts/setlocalversion

info "Creating archive linux-${version}.tar.bz2..."
cd "${temp_dir}"
test $? -eq 0 || error "Changing directory to '${output_dir}' failed"
tar -cjf "${output_dir}/linux-${version}.tar.bz2" "linux-${version}"
test $? -eq 0 || \
  error "Creating bzipped tar archive '${output_dir}/linux-${version}.tar.bz2' failed"

info "Done"

rm -R "${temp_dir}"

exit 0

