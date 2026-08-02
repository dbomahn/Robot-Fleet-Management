FROM python:3.12-slim

WORKDIR /app
COPY pyproject.toml README.md ./
COPY src ./src
RUN python -m pip install --no-cache-dir .

# The service command is a placeholder until transport wiring is implemented.
CMD ["python", "-m", "collision_monitor", "--help"]

