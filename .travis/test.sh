#!/bin/bash

set -o errexit
set -o verbose

scriptdir=$(dirname "${0}")

source /opt/ros/${ROS_DISTRO}/setup.bash

cd /catkin_ws

CMI_OPTION="--install-space /opt/ros/${ROS_DISTRO} --install"

catkin_make_isolated $CMI_OPTION || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make``` failed'; false)
catkin_make_isolated $CMI_OPTION --catkin-make-args tests || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make tests``` failed'; false)
catkin_make_isolated $CMI_OPTION --catkin-make-args run_tests || \
  (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" '```catkin_make run_tests``` failed'; false)

if [ catkin_test_results ];
then
  result_text="
\`\`\`
$(catkin_test_results --all | grep -v Skipping || true)
\`\`\`
"
else
  result_text="
\`\`\`
$(catkin_test_results --all || true)
\`\`\`
$(find build/test_results/ -name *.xml | xargs -n 1 -- bash -c 'echo; echo \#\#\# $0; echo; echo \\\`\\\`\\\`; xmllint --format $0; echo \\\`\\\`\\\`;')
"
fi
catkin_test_results || (gh-pr-comment "${BUILD_LINK} FAILED on ${ROS_DISTRO}" "Test failed$result_text"; false)

gh-pr-comment "${BUILD_LINK} PASSED on ${ROS_DISTRO}" "All tests passed$result_text"

cd ..
rm -rf /catkin_ws || true
