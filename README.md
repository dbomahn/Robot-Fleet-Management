# Collision Monitor

A headless service that makes one deterministic Pause or Resume decision per
robot at each decision tick.

The core decision engine is independent of RabbitMQ. Transport, state
aggregation, geometry, optimisation and simulation are separate modules.

## Requirementss

- Python 3.11 or 3.12
- Shapely 2.x for geometry


## Development

Create and activate a virtual environment, then install the package:

```console
python -m pip install -e '.[dev]'
```


The command-line interface is currently a placeholder:

```console
python -m collision_monitor --help
```

