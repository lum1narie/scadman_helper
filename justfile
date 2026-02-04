# Build the project
build:
  cargo build

# Run the project
run *args:
  cargo run -- {{ args }}

# Build the project (release)
build-release:
  cargo build -r

# Run the project (release)
run-release *args:
  cargo run -r -- {{ args }}

# Fix things in clippy
clippy-fix:
  cargo clippy --all-targets --no-deps --fix --allow-staged

# Format all files
fmt:
  cargo fmt
  taplo format
  nixpkgs-fmt .
  -markdownlint-cli2 --fix .

# Lint the code
lint:
  cargo clippy --all-targets --no-deps

# Check formatting and lint
check: lint
  cargo fmt -- --check
  nixpkgs-fmt --check .
  markdownlint-cli2 --fix .

