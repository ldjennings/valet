{
  description = "Valet_Assignment Python dev environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.${system}.default =
        pkgs.buildFHSEnv {
          name = "valet-dev";

          targetPkgs = pkgs: (with pkgs; [
            python314
            python314Packages.virtualenv
            zsh
            eza
            fd
            pkg-config
            openssl

          ]);

          runScript = "zsh";
        };
    };
}
