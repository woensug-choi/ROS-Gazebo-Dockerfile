#!/usr/bin/env bash
set -Eeuo pipefail

#
# Copyright (C) 2020 IOES authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Determine the parent directory of this script, no matter how it is invoked.
cd "$(dirname "$(readlink -f "$BASH_SOURCE")")/.."

image_name="ioes-ros-gazebo"

if docker info -f '{{ range $key, $value := .Runtimes }}{{ $key }}{{ end }}' | grep nvidia > /dev/null 2>&1; then
    image_name="$image_name-nvidia"
fi

image_plus_tag=$image_name:$(export LC_ALL=C; date +%Y_%m_%d_%H%M)
docker build -t "$image_plus_tag" -t "$image_name:latest" -f "$image_name.dockerfile "$@" .
echo "Built $image_plus_tag and tagged as $image_name:latest"