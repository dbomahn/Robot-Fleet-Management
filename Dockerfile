# syntax=docker/dockerfile:1.7@sha256:a57df69d0ea827fb7266491f2813635de6f17269be881f696fbfdf2d83dda33e

FROM python:3.12.13-slim@sha256:57cd7c3a7a273101a6485ba99423ee568157882804b1124b4dd04266317710de AS builder

ENV PIP_DISABLE_PIP_VERSION_CHECK=1

WORKDIR /build

COPY pyproject.toml README.md ./
COPY src ./src

# Build the application and runtime dependency closure independently of test
# tooling, keeping the default production build lean.
RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip wheel --wheel-dir /runtime-wheels .


FROM builder AS test-builder

# The test target is opt-in. Runtime wheels are offered as local candidates so
# pip can reuse them while resolving the development extras.
RUN --mount=type=cache,target=/root/.cache/pip \
    python -m pip wheel \
        --wheel-dir /test-wheels \
        --find-links=/runtime-wheels \
        ".[dev]"


FROM python:3.12.13-slim@sha256:57cd7c3a7a273101a6485ba99423ee568157882804b1124b4dd04266317710de AS runtime

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

RUN groupadd --gid 10001 collision-monitor \
    && useradd \
        --uid 10001 \
        --gid collision-monitor \
        --create-home \
        --shell /usr/sbin/nologin \
        collision-monitor

WORKDIR /app

COPY --from=builder /runtime-wheels /runtime-wheels
RUN python -m pip install \
        --no-index \
        --find-links=/runtime-wheels \
        collision-monitor \
    && rm -rf /runtime-wheels

USER collision-monitor:collision-monitor

STOPSIGNAL SIGTERM

ENTRYPOINT ["python", "-m", "collision_monitor"]
CMD ["run"]


FROM runtime AS test

USER root

COPY --from=test-builder /test-wheels /test-wheels
RUN python -m pip install \
        --no-index \
        --find-links=/test-wheels \
        "collision-monitor[dev]" \
    && rm -rf /test-wheels

COPY --chown=collision-monitor:collision-monitor pyproject.toml .env.example ./
COPY --chown=collision-monitor:collision-monitor \
    Dockerfile docker-compose.yml .dockerignore ./
COPY --chown=collision-monitor:collision-monitor tests ./tests
COPY --chown=collision-monitor:collision-monitor scenarios ./scenarios

USER collision-monitor:collision-monitor

ENTRYPOINT ["python", "-m", "pytest"]
CMD []
