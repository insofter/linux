#!/bin/sh

src_dir="$1"
export_dir="$2"

cd "${src_dir}"
test $? -eq 0 || exit 1

# version from git tag; it is expected to be in form v<ver>+<tag-ver>,
# e.g. v3.1.4+icdtcp3-1.0.0
version=`git describe --abbrev=0 | sed -e 's/^[^+]*+\(.*\)$/-\1/'`
test -n "${version}" || exit 1

cd "${export_dir}/scripts"
test $? -eq 0 || exit 1

mv "setlocalversion" "_setlocalversion"
test $? -eq 0 || exit 1
cat "_setlocalversion" | sed -e 's/^local_tag=.*$/local_tag="'"${tag}"'"/' \
  -e '/sed:package {/,/sed:package }/ d' > "setlocalversion"
test $? -eq 0 || exit 1
chmod a+x "setlocalversion"
test $? -eq 0 || exit 1
rm "_setlocalversion"
test $? -eq 0 || exit 1

exit 0

