# Convenience wrappers around docker compose.
#
# Run from the repo root:  make -C docker <target>
# Or from this directory:  make <target>

SHELL := /bin/bash
COMPOSE := docker compose -f docker-compose.yml
UID := $(shell id -u)
GID := $(shell id -g)
export UID
export GID

.PHONY: help build rebuild up down shell run gpu sim flash test lint prune

help:   ## Show this help
	@awk 'BEGIN{FS=":.*## "} /^[a-zA-Z_-]+:.*## / {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

build:  ## Build the dev image (CPU-only)
	$(COMPOSE) build dev

build-gpu: ## Build the GPU image
	$(COMPOSE) --profile gpu build dev-gpu

rebuild: ## Rebuild without cache
	$(COMPOSE) build --no-cache dev

shell:  ## Open a shell in a throwaway dev container
	$(COMPOSE) run --rm --service-ports dev bash

up:     ## Start dev container in background
	$(COMPOSE) up -d dev

down:   ## Stop background containers
	$(COMPOSE) down

gpu:    ## Shell in a GPU-enabled container (needs NVIDIA Container Toolkit)
	$(COMPOSE) --profile gpu run --rm --service-ports dev-gpu bash

sim:    ## Shell with sim profile (MAVLink ports forwarded)
	$(COMPOSE) --profile sim run --rm --service-ports sim bash

flash:  ## Flash firmware via probe-rs (requires USB passthrough on host)
	$(COMPOSE) run --rm dev \
	    bash -c "cd /workspace && cargo build -p app-copter --target thumbv7em-none-eabihf --release \
	             && probe-rs run --chip STM32H753IIKx target/thumbv7em-none-eabihf/release/app-copter"

test:   ## Run host-side tests inside the container
	$(COMPOSE) run --rm dev bash -c "cd /workspace && cargo test --workspace"

lint:   ## Run fmt + clippy inside the container
	$(COMPOSE) run --rm dev \
	    bash -c "cd /workspace && cargo fmt --all -- --check && cargo clippy --workspace --all-targets -- -D warnings"

prune:  ## Remove container + volumes (keeps image)
	$(COMPOSE) down -v
