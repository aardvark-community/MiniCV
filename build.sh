#!/bin/sh
dotnet tool restore
dotnet paket restore
dotnet build src/MiniCV.sln 