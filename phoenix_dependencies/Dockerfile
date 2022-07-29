# syntax=docker/dockerfile:experimental
ARG BUILDER_SUFFIX=
ARG BUILDER_PREFIX=
ARG ROS_DISTRO=
FROM ros:${ROS_DISTRO}-ros-core as base
FROM ${BUILDER_PREFIX}builder${BUILDER_SUFFIX} as builder

FROM base as pre_build
COPY * /root/ws/src/
ARG ROSINSTALL_CI_JOB_TOKEN=
ENV ROSINSTALL_CI_JOB_TOKEN $ROSINSTALL_CI_JOB_TOKEN
ARG CI_JOB_TOKEN=
ENV CI_JOB_TOKEN $CI_JOB_TOKEN
ARG CMAKE_ARGS=
ENV CMAKE_ARGS $CMAKE_ARGS
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    /builder/workspace.bash install_from_rosinstall /root/ws/src/dep.repo /root/ws/src/
RUN cd /root/ws/src/phoenix_dependencies/plcnext_deps/ && \
    /root/ws/src/phoenix_dependencies/plcnext_deps/dep_copy.sh ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

FROM pre_build as build
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    /builder/workspace.bash build_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM build as test
RUN --mount=type=bind,from=builder,target=/builder \
    apt-get update -qq && \
    /builder/workspace.bash test_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM pre_build as deploy
RUN --mount=type=bind,from=test,target=/root/ws,source=/root/ws \
    apt-get update -qq && \
    rm -rf /var/lib/apt/lists/*
