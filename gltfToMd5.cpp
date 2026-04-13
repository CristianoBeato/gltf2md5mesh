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

#include "gltfToMd5.hpp"
#include "md5Model.hpp"
namespace gltf = fastgltf;

//
static constexpr int VERSION = 10; // 1.0

static const char arg_input[] = "--input";  // input gltf
static const char arg_output[] = "--mesh";  // output mesh
static const char arg_forcej[] = "--force-joints";  // force to create joits/weights
static const char arg_forcew[] = "--force-weights";  // foce to create weights
static const char arg_help[] = "--help";   // print the help string

static const char help_string[] =
{
    "gltf2md5mesh\n"
    "\n"
    "\t--input\tinput gltf2 model\n"
    "\t--mesh\toutput mesh (.md5mesh)\n"
    "\t--force-joints\tforce to create a root joint in static model that don't have joints\n"
    "\t--force-weights\tforce to create vertices weights in static model that don't have weights\n"
    "\t--help\tprint this help\n"
};

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

int main( int argc, char* argv[] )
{
    std::filesystem::path inPath;
    std::filesystem::path outPath;
    gltfToMd5 gltfToMd5mesh = gltfToMd5();

    for ( int i = 0; i < argc; i++)
    {
        if ( std::strncmp( argv[i], arg_help, std::strlen(arg_help) ) == 0 )
        {
            std::cout << help_string << std::endl;
            return EXIT_SUCCESS;
        }
        else if ( std::strncmp( argv[i], arg_input, std::strlen( arg_input ) ) == 0 )
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
        
        auto model = gltfToMd5mesh.ConvertGLTFtoMD5( inPath );
        
        outPath.replace_extension( "md5mesh" );

        /// Write the .md5mesh to disk
        /// TODO: command line string should be generated from the actual command line arguments, not hardcoded
        model.SetVersion( VERSION ); /// We current suport version 1.0 of the md5 format, but this can be changed in the future if we need to add new features that can be not compatible with the old version
        model.SetCommandLine( "gltf2md5mesh --input " + inPath.string() + " --mesh " + outPath.string() );
        model.Write( outPath.string() );
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

/*
======================================
gltfToMd5::gltfToMd5
======================================
*/
gltfToMd5::gltfToMd5( void )
{
}

/*
======================================
gltfToMd5::~gltfToMd5
======================================
*/
gltfToMd5::~gltfToMd5( void )
{
}

/*
======================================
gltfToMd5::LoadGLTF
======================================
*/
bool gltfToMd5::LoadGLTF( const std::filesystem::path &in_path )
{
    ///
    fastgltf::Parser parser( supportedExtensions );

    ///
    if( !std::filesystem::exists( in_path ) )
        throw std::runtime_error( "Source File Don't exits" );

    ///
    std::cout << "trying to open " << in_path.c_str() << std::endl;
    gltf::Expected<gltf::MappedGltfFile> file = gltf::MappedGltfFile::FromPath( in_path );
	if (!bool(file)) 
        throw std::runtime_error( gltf::getErrorMessage( file.error() ).data() );

    ///
    auto assets = parser.loadGltf( file.get(), in_path.parent_path(), gltfOptions );
    if ( assets.error() != gltf::Error::None )
        throw std::runtime_error( gltf::getErrorMessage( assets.error()).data() );
    
    /// build skining skeleton
    const auto& skin = assets.get().skins[0];
    if( !LoadJoints( assets.get(), skin ) )
        return false;

    if( !LoadMeshes( assets.get(), skin ) )
        return false;

}

/*
======================================
gltfToMd5::LoadJoints
======================================
*/
bool gltfToMd5::LoadJoints( const gltf::Asset &in_assets, const gltf::Skin& in_skin )
{
    uint32_t i = 0;
    uint32_t jointCount = 0;
    auto nodes = in_assets.nodes;

    jointCount = static_cast<uint32_t>( in_skin.joints.size() );
    m_model.ReserveJoints( jointCount );

    // Descobre parent de cada node
    std::vector<int> parent( nodes.size(), -1 );
    for ( i = 0; i < nodes.size(); i++) 
    {
        for (auto c : nodes[i].children)
            parent[c] = (int)i;
    }

    for ( i = 0; i < jointCount; i++) 
    {
        uint32_t nodeIndex = in_skin.joints[i];
        const gltf::Node& node = nodes[nodeIndex];

        MD5::Joint_t joint;

        /// if no joint name, "create" one
        if ( node.name.empty() )
            joint.name =  "joint_" + std::to_string(i);
        else
            joint.name = node.name.c_str();

        joint.parentIndex = parent[nodeIndex];
        
        auto transform = std::get<gltf::TRS>( node.transform );

        /// get translation
        joint.pos = glm::vec3( transform.translation.x(), transform.translation.y(), transform.translation.z() );
 
        /// get rotation
        joint.orient = glm::quat( transform.rotation.w(), transform.rotation.x(), transform.rotation.y(), transform.rotation.z());
        
        /// get scaling ( not used in MD5Mesh )
        // glm::vec3 s = glm::vec3( transform.scale.x(), transform.scale.y(), transform.scale.z() );

        m_model.AddJoint( joint );
    }

    return true;
}

/*
======================================
gltfToMd5::LoadTriangles
======================================
*/
bool gltfToMd5::LoadTriangles(const gltf::Asset &in_assets)
{
    // We specify GenerateMeshIndices, so we should always have indices
	assert( in_prim.indicesAccessor.has_value() ); 
    
    return false;
}

/*
======================================
gltfToMd5::LoadVertexes
======================================
*/
bool gltfToMd5::LoadVertexes(const gltf::Asset &in_assets, const gltf::Primitive &in_prim )
{
    const gltf::Attribute* posAcc = in_prim.findAttribute("POSITION");
    
    // we only try to load the first uv mesh
    const gltf::Attribute* texcoordAcc = in_prim.findAttribute("TEXCOORD_0");
    
    if( posAcc == in_prim.attributes.end() || texcoordAcc == in_prim.attributes.end() )
       return false;

    // Position
    const gltf::Accessor& positionAccessor = in_assets.accessors[posAcc->accessorIndex];
    if (!positionAccessor.bufferViewIndex.has_value())
        return false;    

    // Tex coord
	const gltf::Accessor& texCoordAccessor = in_assets.accessors[texcoordAcc->accessorIndex];
    if (!texCoordAccessor.bufferViewIndex.has_value())
        return false;

    // TODO:
    if( positionAccessor.count != texCoordAccessor.count )
    {
        // TODO: check if he have the same number of the two
    }

    // resize vertex array
    m_positions.resize( positionAccessor.count );
    m_coordinates.resize( texCoordAccessor.count );

    // position data acess lambda
    auto getpos = [&]( fastgltf::math::fvec3 pos, std::size_t idx ) 
    {
		m_positions[m_currentMesh][idx] = glm::vec4( pos.x(), pos.y(), pos.z(), 1.0f );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec3>( in_assets, positionAccessor, getpos );
 
    m_coordinates.resize( texCoordAccessor.count );

    // coordinate data acess lambda
    auto getuv = [&]( fastgltf::math::fvec2 uv, std::size_t idx ) 
    {
	    out_coordinates[idx] = glm::vec2( uv.x(), uv.y() );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec2>( in_assets, texCoordAccessor, getuv );
    return true;
}

/*
======================================
gltfToMd5::LoadWeights
======================================
*/
bool gltfToMd5::LoadWeights( const gltf::Asset &in_assets, const gltf::Primitive& in_prim, std::vector<float> &out_weights, std::vector<uint16_t> &out_joints, uint32_t &out_jointPerVertex )
{
    uint32_t windex = 0;
    uint32_t jindex = 0;

    // we only try to load the first weights influence
    const gltf::Attribute* weightsAcc0 = in_prim.findAttribute("WEIGHTS_0");
    const gltf::Attribute* jointsAcc0 = in_prim.findAttribute("JOINTS_0");
    const gltf::Attribute* weightsAcc1 = in_prim.findAttribute("WEIGHTS_1"); // check if we have 8 weights per vertex
    const gltf::Attribute* jointsAcc1 = in_prim.findAttribute("JOINTS_1");

    /// no joint 
    if( weightsAcc0 == in_prim.attributes.end() && jointsAcc0 == in_prim.attributes.end() )
        return false;

    // vertex weights and joints index
    const gltf::Accessor& weightAccessor0 = in_assets.accessors[weightsAcc0->accessorIndex];
    const gltf::Accessor& jointsAccessor0 = in_assets.accessors[jointsAcc0->accessorIndex];
    const gltf::Accessor& weightAccessor1 = in_assets.accessors[weightsAcc1->accessorIndex];
    const gltf::Accessor& jointsAccessor1 = in_assets.accessors[jointsAcc1->accessorIndex];

    /// no weigts or joints found 
    if (!weightAccessor0.bufferViewIndex.has_value() && !weightAccessor1.bufferViewIndex.has_value() )
        return false;
        
    // resize the weights array 
    if( weightsAcc1 != in_prim.attributes.end() )
    {
        out_weights.resize( weightAccessor0.count + weightAccessor1.count );
        out_jointPerVertex = 8;
    }
    else
    {
        out_weights.resize( weightAccessor0.count );
        out_jointPerVertex = 4;
    }

    // resize the joint array
    if( jointsAccessor1.bufferViewIndex.has_value() )
        out_joints.reserve( jointsAccessor0.count + jointsAccessor1.count );
    else
        out_joints.resize( jointsAccessor0.count );

    // coordinate weight acess lambda
    auto Getweight = [&]( fastgltf::math::fvec4 weight, std::size_t idx ) 
    {   
        for (size_t w = 0; w < 4; w++)
        {
            out_weights[windex + w] = weight[w];
        }        
        windex += 4;
    };

    // coordinate joint acess lambda
    auto Getjoint = [&]( fastgltf::math::u16vec4 joint, std::size_t idx ) 
    {
        for (size_t wj = 0; wj < 4; wj++)
        {
            out_joints[jindex + wj] = joint[wj];
        }        
        jindex += 4;
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec4>( in_assets, weightAccessor0, Getweight );
    fastgltf::iterateAccessorWithIndex<fastgltf::math::u16vec4>( in_assets, jointsAccessor0, Getjoint );

    ///
    if (weightAccessor1.bufferViewIndex.has_value() && jointsAccessor1.bufferViewIndex.has_value() )
    {
        fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec4>( in_assets, weightAccessor1, Getweight );
        fastgltf::iterateAccessorWithIndex<fastgltf::math::u16vec4>( in_assets, jointsAccessor1, Getjoint );
    }    

    return true;
}

/*
======================================
gltfToMd5::LoadInverseMatrixes
======================================
*/
bool gltfToMd5::LoadInverseMatrixes( const gltf::Asset &in_assets, const gltf::Skin& in_skin, std::vector<glm::mat4> &out_inversePose )
{
    const gltf::Accessor& inverseAccessor = in_assets.accessors[*in_skin.inverseBindMatrices];
    if (!inverseAccessor.bufferViewIndex.has_value())
        return false;

    out_inversePose.resize( inverseAccessor.count );

    // inverse bind matrix 
    auto getInverse = [&]( fastgltf::math::fmat4x4 mat, std::size_t idx ) 
    {
		out_inversePose[idx] = glm::make_mat4( mat.data() );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fmat4x4>( in_assets, inverseAccessor, getInverse );

    return true;
}

/*
======================================
gltfToMd5::LoadMesh
======================================
*/
void gltfToMd5::LoadMeshes( const gltf::Asset &in_assets, const gltf::Mesh& in_gltfMesh, const gltf::Skin& in_skin, MD5::Model& out_model )
{
    m_model.ReserveMeshes( in_assets.meshes.size() );

    /// load model meshes
    for ( const auto& m : in_assets.meshes )
    {
        for ( auto& prim : in_gltfMesh.primitives )
        {
            size_t vertexCount = 0;
            uint32_t jointPerVertex = 0;
            std::vector<glm::vec4>      positions = std::vector<glm::vec4>();   // vertex positions ( vec4 for easy joit pos calculation )
            std::vector<glm::vec2>      coordinates = std::vector<glm::vec2>(); // vertex uv 
            std::vector<float>          weights = std::vector<float>();     // joints influences
            std::vector<uint16_t>       joints = std::vector<uint16_t>();   // joint ids
            std::vector<glm::mat4>      inverseBindMatrix = std::vector<glm::mat4>(); // inverse pose matrix

            MD5::Mesh_t subset{};

            if ( prim.materialIndex.has_value() )
            {
                const auto &mtr = in_assets.materials[*prim.materialIndex];
                subset.shader = mtr.name.c_str();
            }
            else
            {
                subset.shader = "defaut";
            }

            // get vertex positions
            if( !LoadPositions( in_assets, prim, positions ) )
                throw std::runtime_error( "A mesh primitive is required to hold the POSITION attribute." );

            // get texture coordinates
            if( !LoadTextureCoordinates( in_assets, prim, coordinates ) )
                throw std::runtime_error( "A mesh primitive is required to hold atleast TEXCOORD_0 attribute." );

            // get vertex weights influences
            if( !LoadWeights( in_assets, prim, weights, joints, jointPerVertex ) )
                throw std::runtime_error( "A mesh primitive is required to hold the WEIGHTS_0 attribute." );

            // inverse pose Bind Matrix
            if( !LoadInverseMatrixes( in_assets, in_skin, inverseBindMatrix ) )
                throw std::runtime_error( "A mesh primitive is required to hold the WEIGHTS_0 attribute." );

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

    #if 0 // TODO: fix
            if( weights.size() != joints.size() )
            {

            }

            if( weights.size() == 0 )
            {
                weights.resize( positions.size() );
                for (size_t i = 0; i < weights.size(); i++)
                {
                    weights[i] = 1.0f;
                }
            }
            else if( weights.size() != positions.size() )
                throw std::runtime_error( "weights count != from vertex position count" );

            if(  == 0 )
            {
                joints.resize( positions.size() );
                for (size_t i = 0; i < joints.size(); i++)
                {
                    joints[i] = glm::u16vec4( 0, 0, 0, 0 );
                }
            }
            else if( joints.size() != positions.size() )
                throw std::runtime_error( "vertex joint count != from vertex position count" );
    #endif 
        
            vertexCount = positions.size();
            subset.vertices.resize(vertexCount);

            // MD5 weights são uma lista linear + startWeight
            std::vector<MD5::Weight_t> weightsList;

            uint32_t nextWeight = 0;

            // Transformar para MD5
            for ( uint32_t i = 0; i < vertexCount; i++) 
            {
                MD5::Vertex_t v{};
                v.uv = coordinates[i];
                v.startWeight = nextWeight;
                
                for ( int w = 0; w < jointPerVertex; w++, nextWeight++) 
                {
                    MD5::Weight_t mw{};
                    mw.joint = joints[nextWeight];
                    mw.bias  = weights[nextWeight];
                    
                    /// ignore 0 bias weights 
                    if( mw.bias == 0 )
                        continue;
                    
                    // weight in the joint space 
                    mw.pos = inverseBindMatrix[joints[nextWeight]] * positions[i];
                    
                    weightsList.push_back(mw);
                }
                
                v.weightCount = nextWeight - v.startWeight ;
                subset.vertices[i] = v;
            }

            subset.weights = std::move(weightsList);

            // Índices
            if ( prim.indicesAccessor ) 
            {
                auto& indexAcc = in_assets.accessors[*prim.indicesAccessor];
                std::vector<uint16_t> indices;
                indices.resize( indexAcc.count );
                
                if ( indexAcc.componentType == gltf::ComponentType::UnsignedByte || indexAcc.componentType == gltf::ComponentType::UnsignedShort )
                    gltf::copyFromAccessor<std::uint16_t>( in_assets, indexAcc, indices.data());

                for (size_t i = 0; i < indexAcc.count; i += 3 )
                {
                    MD5::Triangle_t t{};
                    t.v0 = indices[ i + 0 ];
                    t.v1 = indices[ i + 1 ];
                    t.v2 = indices[ i + 2 ];
                    subset.triangles.push_back(t);
                }
            }
            else
                throw std::runtime_error( " not valide index on model " );

            out_model.AddMesh( subset );
        }

        m_currentMesh++;
    }
}
