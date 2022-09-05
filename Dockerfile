# syntax=docker/dockerfile:experimental
ARG BUILDER_SUFFIX=
ARG BUILDER_PREFIX=
ARG DEP_PREFIX=
ARG DEP_SUFFIX=
ARG ROS_DISTRO=
FROM ${DEP_PREFIX}phoenix_dependencies_${ROS_DISTRO}${DEP_SUFFIX} as base
FROM ${BUILDER_PREFIX}builder${BUILDER_SUFFIX} as builder

FROM base as pre_build
ARG CMAKE_ARGS=
ENV CMAKE_ARGS $CMAKE_ARGS
COPY . /root/ws/src/phoenix_bridge/
COPY --from=builder workspace.bash /builder/workspace.bash
RUN apt-get update -qq && \
    /builder/workspace.bash update_list /root/ws && \
    rm -rf /var/lib/apt/lists/*

FROM pre_build as build
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
