# root directory of the project
rm -rf .git/
rm .gitignore
rm .gitmodules
rm README.md

# remove hmi source files
rm -rf src/hmi/customControl
rm -rf src/hmi/include/
rm -rf src/hmi/msg/
rm -rf src/hmi/privateHeader/
rm -rf src/hmi/src/
rm src/hmi/CMakeLists.txt

# remove hmi_api source files
rm -rf src/hmi_api/include/
rm -rf src/hmi_api/msg/
rm -rf src/hmi_api/srv/
rm src/hmi_api/CMakeLists.txt
rm src/hmi_api/README.md