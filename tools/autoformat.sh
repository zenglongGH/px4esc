#!/bin/bash
#
# Autoformatter
#

source_files=`find ../src -type f -name "*.[ch]" | grep -v cmsis | grep -v ARM`
echo 'Source files:'
printf '%s\n' "$source_files"

# Uncrustify must be invoked twice in order for all modificators to work correctly
for i in {1..2}
do
    echo 'Running uncrustify...'
    uncrustify --replace --no-backup -c px4_uncrustify.cfg $source_files
done

echo 'Running sed...'
for f in $source_files
do
    # Removing leading whitespaces before preprocessor directives
    sed -i 's/^[\t\ ]*#/#/g' $f
    # Removing extra spaces after '#include' directives
    sed -i 's/#include[\t\ ]*/#include /g' $f
done
