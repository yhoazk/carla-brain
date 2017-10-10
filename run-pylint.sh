#!/bin/bash

while getopts "h" arg; do
  case $arg in
    h)
      echo usage: $(basename $0) \[\<pathspec\>\]
      exit 0
      ;;
  esac
done

running_in_docker() {
    awk -F/ '$2 == "docker"' /proc/self/cgroup | read
}

# pylint for all *.py files
function _pylint {
if [[ $1 == *py ]]
then
    echo "###### check: $1"
    pylint --output-format=parseable --rcfile=$SCRIPTPATH/.pylintrc "$1"
fi
}

function _write {
    file=$(readlink -e "$1")
    echo "$file" >> $LOCKFILE
}

if running_in_docker
then
    pip install pylint > /dev/null
fi

SCRIPTPATH=$(dirname "$(readlink -f "$0")")
LOCKFILE=$(mktemp)
trap "{ rm -f $LOCKFILE; }" EXIT

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
            _write "$FOO"
        fi
        if [ -d $FOO ]
        then
            echo add recursive $FOO $(find $FOO -name "*.py")
            for file in $(find $FOO -name "*.py")
            do
                _write "$file"
            done
        fi
    fi
    if [[ $a != ??* ]] && [[ $a != !!* ]] && [[ $a != D* ]]
    then
        _write "$FOO"
    fi
done

# Find all changed files up to commit $1 and test them
if [ -n $1 ]
then
    git diff --name-only $1 | while read -r line;
    do
        _write "$SCRIPTPATH/$line" 
    done
fi

sort -u $LOCKFILE | while read -r line;
do
  _pylint $line
done
