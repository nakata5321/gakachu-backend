{ lib 
, mkRosPackage
, robonomics_comm-nightly
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "gakachu_backend";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm-nightly python3Packages.fastapi python3Packages.uvicorn ];

  meta = with lib; {
    description = "Gakachu backend";
    homepage = http://github.com/vourhey/gakachu_frontend;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey, nakata5321 ];
  };
}

