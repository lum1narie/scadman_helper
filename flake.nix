{
  description = "A Rust project with Nix Flakes";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
    flake-utils.url = "github:numtide/flake-utils";
    crane.url = "github:ipetkov/crane";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, crane, rust-overlay, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };
        rustVersion = "1.91.1";

        rustToolchain = pkgs.rust-bin.stable.${rustVersion}.default;
        craneLib = (crane.mkLib pkgs).overrideToolchain rustToolchain;
        src = pkgs.lib.cleanSource ./.;

        # Common arguments for all builds
        commonArgs = {
          inherit src;
          # pname = "";
          version = "0.1.0";
        };

        # Build the Rust project
        cargoArtifacts = craneLib.buildDepsOnly commonArgs;
        # For release builds (nix build)
        releaseBuild = craneLib.buildPackage (commonArgs // {
          inherit cargoArtifacts;
        });

      in
      {
        formatter = pkgs.nixpkgs-fmt;

        packages = {
          default = releaseBuild;
        };

        apps = {
          default = flake-utils.lib.mkApp {
            drv = self.packages.${system}.default;
          };
        };

        checks = {
          inherit releaseBuild;

          clippy = craneLib.cargoClippy (
            commonArgs
            // {
              inherit cargoArtifacts;
              cargoClippyExtraArgs = "--all-targets";
            }
          );

          doc = craneLib.cargoDoc (
            commonArgs
            // {
              inherit cargoArtifacts;
            }
          );

          # Check formatting
          rust-fmt = craneLib.cargoFmt {
            inherit src;
          };

          toml-fmt = craneLib.taploFmt {
            src = pkgs.lib.sources.sourceFilesBySuffices src [ ".toml" ];
          };

          nextest = craneLib.cargoNextest (
            commonArgs
            // {
              inherit cargoArtifacts;
              partitions = 1;
              partitionType = "count";
              cargoNextestPartitionsExtraArgs = "--no-tests=pass";
            }
          );
        };

        devShells.default = craneLib.devShell {
          checks = self.checks.${system};
          packages = with pkgs; [
            just
            markdownlint-cli2
            nixpkgs-fmt
            taplo
          ];
        };

      }
    );
}
