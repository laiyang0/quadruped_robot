#!/usr/bin/env bash

# This file must be sourced so that the environment changes affect the
# current terminal.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Usage: source /home/laiyang/quadruped_robot/new.sh" >&2
    exit 1
fi

# Avoid mixing an active Conda environment with ROS 2. In non-interactive
# shells the conda executable may exist without the shell function required
# by `conda deactivate`, so load its shell hook first when necessary.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    if ! declare -F conda >/dev/null; then
        conda_executable="${CONDA_EXE:-$(type -P conda 2>/dev/null)}"
        conda_root="${conda_executable%/bin/conda}"
        conda_hook="${conda_root}/etc/profile.d/conda.sh"

        if [[ -f "${conda_hook}" ]]; then
            source "${conda_hook}"
        fi
    fi

    if declare -F conda >/dev/null; then
        conda deactivate || return 1
    else
        echo "Unable to deactivate Conda: conda shell hook was not found." >&2
        return 1
    fi
fi

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_setup="${workspace_root}/install/setup.bash"

if [[ ! -f "${workspace_setup}" ]]; then
    echo "Workspace setup file not found: ${workspace_setup}" >&2
    echo "Build the workspace first with: colcon build --symlink-install" >&2
    unset workspace_root workspace_setup conda_executable conda_root conda_hook
    return 1
fi

source "${workspace_setup}"
setup_status=$?

unset workspace_root workspace_setup conda_executable conda_root conda_hook
return "${setup_status}"
