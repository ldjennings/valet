{
  description = "Valet Assignment - Python simulation dev shell";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      devShells.${system}.default = pkgs.mkShell {
        # NixOS system requirements (add to your NixOS configuration):
        # nix-ld = {
        #   enable = true;
        #   libraries = with pkgs; [
        #     libgcc.lib # provides libstdc++.so.6, needed to get numpy on python working
        #     libx11  # needed for manual mode
        #     libxext # same as above
        #   ];
        # };
        # Without this, pip-installed native packages (numpy, scipy etc.)
        # will fail to load libstdc++.so.6 at runtime.
        packages = with pkgs; [
          python312
          python312Packages.pip
          python312Packages.setuptools
          python312Packages.mypy
          python312Packages.ruff      # linter + formatter, replaces flake8/black/isort
          boost
        ];

        shellHook = ''
          export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH
          export CPLUS_INCLUDE_PATH="${pkgs.boost.dev}/include:$CPLUS_INCLUDE_PATH"
          if [ ! -d .venv ]; then
            python -m venv .venv
          fi
          source .venv/bin/activate
          pip install -e . --quiet
        '';
      };
    };
}