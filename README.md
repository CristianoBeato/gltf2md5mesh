# gltf2md5mesh
A command-line tool for converting glTF 2.0 models to idTech4 meshes (Doom 3 engine).

## About the Project

This tool was born from a recurring problem during my idTech4 development: modeling tools evolve (constant blender bpy evolving), exporters break, plugins stop working, and every update becomes a gamble — “will my exporter survive this time?”

To escape this cycle, I decided to rely on a stable, widely supported transfer format: glTF 2.0, maintained by the Khronos Group. It’s modern, consistent, well-specified, and supported by basically every 3D tool out there.

gltf2md5mesh aims to provide a reliable, long-term bridge between glTF assets and the MD5Mesh format used by idTech4 (Doom 3/Dhelm3/ RBDOOM-3-BFG mods).

## Dependencies

This project relies on two open-source libraries:

fastgltf — for efficient glTF 2.0 parsing

GLM (OpenGL Mathematics) — for vector, matrix, and quaternion math

## Current Features

Right now, the tool includes:

Mesh export (MD5Mesh)
Full conversion of geometry, joints, weights, and mesh structure.

It’s designed to be simple, predictable, and easy to integrate into build pipelines or automation scripts.

## Planned Features

Development is ongoing, and several additions planed for future:

- Animation export (MD5Anim)
- Camera export (MD5Camera)
- Configurable export axis
- Choose which axis should be up: X, Y, or Z (  may not, you can change on export GLTF, but, who knows )
- Support for more than 4 vertex weights per vertex
- Fully supported in glTF and useful for high-quality rigs
- Internal improvements for handling complex hierarchies and larger models

The goal is to turn this tool into a complete and dependable bridge between the glTF and idTech4.

## Purpose

To provide a plugin-independent, future-proof workflow that avoids breaking whenever modeling software changes — keeping the pipeline stable, predictable, and productive.

## Credits

Special thanks to the authors and contributors of:

fastgltf

GLM

These libraries make the project possible.

## License

The gltf2md5meh is licensed under the MIT License.
