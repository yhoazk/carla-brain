#!/bin/bash

while getopts "h" arg; do
  case $arg in
    h)
      echo usage: $(basename $0) \[\<pathspec\>\]
      exit 0
      ;;
  esac
done

# pylint for all *.py files
function _pylint {
if [[ $1 == *py ]]
then
    echo "###### check: $1 (origin: $2)"
    pylint --output-format=parseable --rcfile=._pylintrc $1
fi
}

# Track for changed and untracked files
git status -s | while read -r line;
do
    FOO=$(echo "$line" | awk '{print $2}');
    a=$(echo "$line" | awk '{print $1}');
    if [[ $a == ??* ]]
    then
        if [ -f $FOO ]
        then
            #echo add $FOO
            _pylint $FOO "untracked file"
        fi
        if [ -d $FOO ]
        then
            echo add recursive $FOO $(find $FOO -name "*.py")
            for file in $(find $FOO -name "*.py")
            do
                _pylint $file "untracked directory $FOO"
            done
        fi
    fi
    if [[ $a != ??* ]] && [[ $a != !!* ]] && [[ $a != D* ]]
    then
        _pylint $FOO "git managed and not commited"
        #echo normally add $FOO
    fi
done

# Find all changed files up to commit $1 and test them
if [ -n $1 ]
then
    git diff --name-only $1 | while read -r line;
    do
        _pylint $line "commited file"
    done
fi

