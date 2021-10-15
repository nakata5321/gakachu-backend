{ lib 
, mkRosPackage
, robonomics_comm-nightly
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "gakachu_frontend";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm-nightly python3Packages.fastapi python3Packages.uvicorn ];

  meta = with lib; {
    description = "GakaChu Frontend";
    homepage = http://github.com/vourhey/gakachu_frontend;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}

