/*
===========================================================================================
	This file is part of gltf2md5mesh.
	
	Copyright (c) 2025 - Cristiano B. Santos <cristianobeato_dm@hotmail.com>
	Contributor(s): none yet.

-------------------------------------------------------------------------------------------
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
===========================================================================================
*/

#include "precompiled.hpp"
#include "gltfToMd5.hpp"

//
static constexpr int VERSION = 10; // 1.0

namespace gltf = fastgltf;

static const char arg_input[] = "--input";
static const char arg_output[] = "--output";

static constexpr auto supportedExtensions =
			fastgltf::Extensions::KHR_mesh_quantization |
			fastgltf::Extensions::KHR_texture_transform |
			fastgltf::Extensions::KHR_materials_variants;
    
static constexpr auto gltfOptions =
            fastgltf::Options::DontRequireValidAssetMember |
            fastgltf::Options::AllowDouble |
            fastgltf::Options::LoadExternalBuffers |
            fastgltf::Options::LoadExternalImages |
			fastgltf::Options::GenerateMeshIndices;

static void LoadJoints(const gltf::Asset& asset, const gltf::Skin& skin, MD5Model_t& out);
static void LoadMesh( const gltf::Asset& asset, const gltf::Mesh& gltfMesh, const gltf::Skin& skin, MD5Model_t& outModel );
static MD5Model_t ConvertGLTFtoMD5( const std::filesystem::path& in_path );
static void SaveMD5( const std::filesystem::path& in_path, const std::string &in_cmdline, const MD5Model_t &in_model );

int main( int argc, char* argv[] )
{
    std::filesystem::path inPath;
    std::filesystem::path outPath;

    for ( int i = 0; i < argc; i++)
    {
        if ( std::strncmp( argv[i], arg_input, std::strlen( arg_input ) ) == 0 )
        {
            if ( ( i + 1 ) >= argc )
                break;

            inPath = argv[i + 1];
        }
        else if ( std::strncmp( argv[i], arg_output, std::strlen( arg_output ) ) == 0 )
        {
             if ( ( i + 1 ) >= argc )
                break;

            outPath = argv[i + 1];
        }
    }
    
   
    try
    {
        if ( outPath.empty() )
            outPath = inPath;
        
        auto model = ConvertGLTFtoMD5( inPath );
        
        outPath.replace_extension( "md5mesh" );
        SaveMD5( outPath,"just test... for now", model );
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    
    return 0;
}

void LoadJoints(const gltf::Asset& asset, const gltf::Skin& skin, MD5Model_t& out)
{
    out.joints.reserve(skin.joints.size());

    // Descobre parent de cada node
    std::vector<int> parent(asset.nodes.size(), -1);
    for (size_t i = 0; i < asset.nodes.size(); i++) 
    {
        for (auto c : asset.nodes[i].children)
            parent[c] = (int)i;
    }

    for (size_t i = 0; i < skin.joints.size(); i++) 
    {
        uint32_t nodeIndex = skin.joints[i];
        const gltf::Node& node = asset.nodes[nodeIndex];

        MD5Joint_t j{};

        if ( node.name.empty() )
            j.name =  "joint_" + std::to_string(i);
        else
            j.name = node.name.c_str();

        j.parentIndex = parent[nodeIndex];
        // Coleta transform
        glm::vec3 t(0);
        glm::quat r( 1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 s(1);

        auto transform = std::get<gltf::TRS>( node.transform );
        t = glm::vec3( transform.translation.x(), transform.translation.y(), transform.translation.z() );
        r = glm::quat( transform.rotation.w(), transform.rotation.x(), transform.rotation.y(), transform.rotation.z());
        s = glm::vec3( transform.scale.x(), transform.scale.y(), transform.scale.z() );

        // MD5 usa orientação como vec3, faltando ‘w’
        glm::quat q = r;  
        j.orient = glm::vec3(q.x, q.y, q.z);

        j.pos = t;

        out.joints.push_back(j);
    }
}

static void LoadPositions( const gltf::Asset& asset, const gltf::Primitive& prim, std::vector<glm::vec4> &positions )
{
    const gltf::Attribute* posAcc = prim.findAttribute("POSITION");
	assert( prim.indicesAccessor.has_value() ); // We specify GenerateMeshIndices, so we should always have indices
    
    // 
    if ( posAcc == prim.attributes.end() )
        throw std::runtime_error( "A mesh primitive is required to hold the POSITION attribute." );

    // Position
    const gltf::Accessor& positionAccessor = asset.accessors[posAcc->accessorIndex];
    if (!positionAccessor.bufferViewIndex.has_value())
        throw std::runtime_error( "" );

    // resize vertex array
    positions.resize( positionAccessor.count );

    // position data acess lambda
    auto getpos = [&]( fastgltf::math::fvec3 pos, std::size_t idx ) 
    {
		positions[idx] = glm::vec4( pos.x(), pos.y(), pos.z(), 1.0f );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec3>(asset, positionAccessor, getpos );
}

static void LoadTextureCoordinates( const gltf::Asset& asset, const gltf::Primitive& prim, std::vector<glm::vec2> &coordinates )
{
    // we only try to load the first uv mesh
    const gltf::Attribute* texcoordAcc = prim.findAttribute("TEXCOORD_0");
    if( texcoordAcc != prim.attributes.end())
    {
        // Tex coord
		const gltf::Accessor& texCoordAccessor = asset.accessors[texcoordAcc->accessorIndex];
        if (!texCoordAccessor.bufferViewIndex.has_value())
            return;
        
        coordinates.resize( texCoordAccessor.count );

        // coordinate data acess lambda
        auto getuv = [&]( fastgltf::math::fvec2 uv, std::size_t idx ) 
        {
	    	coordinates[idx] = glm::vec2( uv.x(), uv.y() );
        };

         fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec2>(asset, texCoordAccessor, getuv );
    }
}

static void LoadWeights( const gltf::Asset& asset, const gltf::Primitive& prim, std::vector<glm::vec4> &weights )
{
    // we only try to load the first weights influence
    const gltf::Attribute* weightsAcc = prim.findAttribute("WEIGHTS_0");
    if( weightsAcc != prim.attributes.end())
    {
        // vertex weights
		const gltf::Accessor& weightAccessor = asset.accessors[weightsAcc->accessorIndex];
        if (!weightAccessor.bufferViewIndex.has_value())
            return;
        
        weights.resize( weightAccessor.count );

        // coordinate data acess lambda
        auto getweight = [&]( fastgltf::math::fvec4 weight, std::size_t idx ) 
        {
	    	weights[idx] = glm::vec4( weight.x(), weight.y(), weight.z(), weight.w() );
        };

         fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec4>( asset, weightAccessor, getweight );
    }
}

static void LoadWeightsJoints( const gltf::Asset& asset, const gltf::Primitive& prim, std::vector<glm::u16vec4> &joints )
{
    // we only try to load the first uv mesh
    const gltf::Attribute* jointsAcc = prim.findAttribute("JOINTS_0");
    if( jointsAcc != prim.attributes.end())
    {
        // vertex joints
		const gltf::Accessor& jointsAccessor = asset.accessors[jointsAcc->accessorIndex];
        if (!jointsAccessor.bufferViewIndex.has_value())
            return;
        
        joints.resize( jointsAccessor.count );

        // coordinate data acess lambda
        auto getjoint = [&]( fastgltf::math::u16vec4 joint, std::size_t idx ) 
        {
	    	joints[idx] = glm::u16vec4( joint.x(), joint.y(), joint.z(), joint.w() );
        };

         fastgltf::iterateAccessorWithIndex<fastgltf::math::u16vec4>(asset, jointsAccessor, getjoint );
    }
}

static void LoadInverseMatrixes( const gltf::Asset& asset, const gltf::Skin& skin, std::vector<glm::mat4> &inversePose )
{
    const gltf::Accessor& inverseAccessor = asset.accessors[*skin.inverseBindMatrices];
        if (!inverseAccessor.bufferViewIndex.has_value())
            return;

    inversePose.resize( inverseAccessor.count );

    // inverse bind matrix 
    auto getInverse = [&]( fastgltf::math::fmat4x4 mat, std::size_t idx ) 
    {
		inversePose[idx] = glm::make_mat4( mat.data() );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fmat4x4>( asset, inverseAccessor, getInverse );
}

void LoadMesh( const gltf::Asset& asset, const gltf::Mesh& gltfMesh, const gltf::Skin& skin, MD5Model_t& outModel )
{
    for (auto& prim : gltfMesh.primitives) 
    {
        size_t vertexCount = 0;
        std::vector<glm::vec4>      positions = std::vector<glm::vec4>();   // vertex positions ( vec4 for easy joit pos calculation )
        std::vector<glm::vec2>      coordinates = std::vector<glm::vec2>(); // vertex uv 
        std::vector<glm::vec4>      weights = std::vector<glm::vec4>();     // joints influences
        std::vector<glm::u16vec4>   joints = std::vector<glm::u16vec4>();   // joint ids
        std::vector<glm::mat4>      inverseBindMatrix = std::vector<glm::mat4>(); // inverse pose matrix

        MD5MeshSubset_t subset{};

        // get vertex positions
        LoadPositions( asset, prim, positions );

        // get texture coordinates
        LoadTextureCoordinates( asset, prim, coordinates );

        // get vertex weights influences
        LoadWeights( asset, prim, weights );

        // get vertex joints ids
        LoadWeightsJoints( asset, prim, joints );

        // inverse pose Bind Matrix
        LoadInverseMatrixes( asset, skin, inverseBindMatrix );

        if( coordinates.size() == 0)
        {
            coordinates.resize( positions.size() );
            // made all blanc
            for (size_t i = 0; i < coordinates.size(); i++)
            {
                coordinates[i] = glm::vec2( 0.0f, 0.0f );
            }
        }
        else if( coordinates.size() != positions.size() )
            throw std::runtime_error( "texture coordinate count != from vertex position count" );


        if( weights.size() == 0 )
        {
            weights.resize( positions.size() );
            for (size_t i = 0; i < weights.size(); i++)
            {
                weights[i] = glm::vec4( 1.0f, 0.0f, 0.0f, 0.0f );
            }
        }
        else if( weights.size() != positions.size() )
            throw std::runtime_error( "weights count != from vertex position count" );

        if( joints.size() == 0 )
        {
            joints.resize( positions.size() );
            for (size_t i = 0; i < joints.size(); i++)
            {
                joints[i] = glm::u16vec4( 0, 0, 0, 0 );
            }
        }
        else if( joints.size() != positions.size() )
            throw std::runtime_error( "vertex joint count != from vertex position count" );

        vertexCount = positions.size();
        subset.vertices.resize(vertexCount);

        // MD5 weights são uma lista linear + startWeight
        std::vector<MD5Weight_t> weightsList;

        size_t nextWeight = 0;

        // Transformar para MD5
        for (size_t i = 0; i < vertexCount; i++) 
        {
            MD5Vertex_t v{};
            v.uv = coordinates[i];
            v.startWeight = nextWeight;
            v.weightCount = 4; // glTF garante 4 weights

            for (int w = 0; w < 4; w++) 
            {
                MD5Weight_t mw{};
                mw.joint = joints[i][w];
                mw.bias  = weights[i][w];

                // weight in the joint space 
                mw.pos = inverseBindMatrix[joints[i][w]] * positions[i];

                weightsList.push_back(mw);
                nextWeight++;
            }

            subset.vertices[i] = v;
        }

        subset.weights = std::move(weightsList);

        // Índices
        if ( prim.indicesAccessor ) 
        {
            auto& indexAcc = asset.accessors[*prim.indicesAccessor];
            std::vector<uint16_t> indices;
            indices.resize( indexAcc.count );
            
            if ( indexAcc.componentType == gltf::ComponentType::UnsignedByte || indexAcc.componentType == gltf::ComponentType::UnsignedShort )
                gltf::copyFromAccessor<std::uint16_t>( asset, indexAcc, indices.data());

            for (size_t i = 0; i < indexAcc.count; i += 3 )
            {
                MD5Triangle_t t{};
                t.v0 = indices[ i + 0 ];
                t.v1 = indices[ i + 1 ];
                t.v2 = indices[ i + 2 ];
                subset.triangles.push_back(t);
            }
        }
        else
            throw std::runtime_error( " not valide index om model " );

        outModel.meshes.push_back(std::move(subset));
    }
}

MD5Model_t ConvertGLTFtoMD5( const std::filesystem::path& in_path )
{
    MD5Model_t model{};

    fastgltf::Parser parser( supportedExtensions );

    if( !std::filesystem::exists( in_path ) )
        throw std::runtime_error( "Source File Don't exits" );

    std::cout << "trying to open " << in_path.c_str() << std::endl;
    gltf::Expected<gltf::MappedGltfFile> file = gltf::MappedGltfFile::FromPath( in_path );
	if (!bool(file)) 
        throw std::runtime_error( gltf::getErrorMessage( file.error() ).data() );

    gltf::Expected<gltf::Asset> assets = parser.loadGltf( file.get(), in_path.parent_path(), gltfOptions);
    if ( assets.error() != gltf::Error::None )
        throw std::runtime_error( gltf::getErrorMessage( assets.error()).data() );

    // build skining skeleton
    const auto& skin = assets.get().skins[0];
    LoadJoints( assets.get(), skin, model);

    // load model meshes
    for ( const auto& m : assets.get().meshes )
    {
        LoadMesh( assets.get(), m, skin, model);
    }

    return model;
}

static void SaveMD5( const std::filesystem::path& in_path, const std::string &in_cmdline, const MD5Model_t &in_model )
{
	uint32_t i = 0;
	uint32_t j = 0;
	std::ofstream file( in_path );
    if (!file.is_open()) 
        std::runtime_error( "Erro: failed to write model" ); 

    // never in exponencial
    file << std::fixed << std::setprecision(6);

    //
    file << std::setprecision(6);  // 6 decimal digits (like Doom 3)

	// write file version
	file << "MD5Version " << VERSION << "\n";

	// write commandline
	file << "commandline " << "\"" << in_cmdline << "\"\n\n";

	// write the joints and meshes count
	file << "numJoints " << in_model.joints.size() << "\n";
	file << "numMeshes " << in_model.meshes.size() << "\n\n";

	file << "joints\n{\n";

	for ( i = 0; i < in_model.joints.size(); i++)
	{
		MD5Joint_t joint = in_model.joints[i];
		
		// write joint "name" and parent index ( -1 for root )
		file << "\t\"" << joint.name << "\" " << joint.parentIndex;
		
		// write joint position
		file << " ( " << joint.pos.x << " " << joint.pos.y << " " << joint.pos.z << " ) ";
		
		// write joint orientation
		file << " ( " << joint.orient.x << " " << joint.orient.y << " " << joint.orient.z << " )\n";
	}

	file << "}\n";
	file << std::endl;

	for ( i = 0; i < in_model.meshes.size(); i++)
	{
		MD5MeshSubset_t mesh = in_model.meshes[i];
		file << "mesh\n{\n";

		file << "\tshader \"" << mesh.shader << "\"\n\n";

		file << "\tnumverts " << mesh.vertices.size() << "\n";
		for ( j = 0; j < mesh.vertices.size(); j++ )
		{
			auto vert = mesh.vertices[j];
			file << "\tvert " << j << " ( " << vert.uv.x << " " << vert.uv.y << " ) " << vert.startWeight << " " << vert.weightCount << "\n";
		}
		file << std::endl;

		file << "\tnumtris " << mesh.triangles.size() << "\n";
		for ( j = 0; j < mesh.triangles.size(); j++)
		{
			auto tri = mesh.triangles[j];
			file << "\ttri " << j << " " << tri.v0 << " " << tri.v1 << " " << tri.v2 << "\n";
		}
		file << std::endl;


		file << "\tnumweights " << mesh.weights.size();
		for ( j = 0; j < mesh.weights.size(); j++ )
		{
			auto weight = mesh.weights[j];
			
			// write weight index, joint id, bias
			file << "\tweight " << j << " " << weight.joint << " " << weight.bias << " ";

			// write joint related position
			file << "( " << weight.pos.x << " " << weight.pos.y << " " << weight.pos.z << " )\n"; 
		}
		file << std::endl;

		// mesh end 
		file << "}"<< std::endl;
	}

	file.close();
}
