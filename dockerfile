# syntax=docker/dockerfile:experimental
ARG BUILDER_SUFFIX=
ARG BUILDER_PREFIX=
ARG ROS_DISTRO=
FROM public.ecr.aws/docker/library/ros:${ROS_DISTRO}-ros-core as base
FROM ${BUILDER_PREFIX}builder${BUILDER_SUFFIX} as builder


FROM base as deps
COPY . /root/ws/src/
COPY --from=builder workspace.bash /builder/workspace.bash
ARG ROSINSTALL_CI_JOB_TOKEN=
ENV ROSINSTALL_CI_JOB_TOKEN $ROSINSTALL_CI_JOB_TOKEN
ARG CI_JOB_TOKEN=
ENV CI_JOB_TOKEN $CI_JOB_TOKEN
ARG DEP_REPO_NAME=
ENV DEP_REPO_NAME $DEP_REPO_NAME
ARG DEP_REPO_URL=
ENV DEP_REPO_URL $DEP_REPO_URL
RUN apt-get update -qq && \
    /builder/workspace.bash install_from_rosinstall /root/ws/src/dep.repo /root/ws/src/
RUN cd /root/ws/src/${DEP_REPO_NAME}/plcnext_deps/ && \
    /root/ws/src/${DEP_REPO_NAME}/plcnext_deps/dep_copy.sh ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

FROM deps as pre_build
COPY . /root/ws/src/
RUN apt-get update -qq && \
    /builder/workspace.bash update_list /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM pre_build as build
ARG CMAKE_ARGS=
ENV CMAKE_ARGS $CMAKE_ARGS
RUN apt-get update -qq && \
    /builder/workspace.bash build_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM build as test
RUN apt-get update -qq && \
    /builder/workspace.bash test_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM test as install
RUN apt-get update -qq && \
    /builder/workspace.bash install_workspace /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM base as deploy
COPY --from=install /root/ws/DEPENDS /root/ws/DEPENDS
COPY --from=builder workspace.bash /builder/workspace.bash
RUN apt-get update -qq && \
    /builder/workspace.bash install_depends /root/ws && \
    rm -rf /var/lib/apt/lists/*
COPY --from=install /opt/ros/$ROS_DISTRO /opt/ros/$ROS_DISTRO