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
        
        gltfToMd5mesh.LoadGLTF( inPath.string() );
        
        outPath.replace_extension( "md5mesh" );

        /// Write the .md5mesh to disk
        /// TODO: command line string should be generated from the actual command line arguments, not hardcoded
        // model.SetVersion( VERSION ); /// We current suport version 1.0 of the md5 format, but this can be changed in the future if we need to add new features that can be not compatible with the old version
        // model.SetCommandLine( "gltf2md5mesh --input " + inPath.string() + " --mesh " + outPath.string() );
        // model.Write( outPath.string() );
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
void gltfToMd5::LoadGLTF( const std::filesystem::path &in_path )
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
    LoadJoints( assets.get(), skin );
    LoadMeshes( assets.get(), skin );
}

MD5::Model gltfToMd5::BuildMD5( void )
{
    uint32_t i = 0;
    uint32_t numJoints = 0;
    uint32_t numMeshes = 0;
    MD5::Model md5Model = MD5::Model();

    numJoints = static_cast<uint32_t>( m_joints.size() );
    numMeshes = static_cast<uint32_t>( m_meshes.size() );

    /// 
    md5Model.ReserveJoints( numJoints );
    md5Model.ReserveMeshes( numMeshes );

    for ( i = 0; i < numJoints; i++)
    {
        MD5::Joint_t newJoint{};
        newJoint.name = m_joints[i].name;
        newJoint.parent = m_joints[i].parent;
        newJoint.position = m_joints[i].position;
        auto r = m_joints[i].rotation;
        newJoint.rotation = glm::quat( r.w, r.x, r.y, r.z );
        md5Model.AddJoint( newJoint );
    }
    
    for ( i = 0; i < numMeshes; i++)
    {
        uint32_t nextWeight = 0;
        uint32_t numVertexes = 0;
        uint32_t numTriangles = 0;
        MD5::Mesh_t mesh{};
        auto source_mesh = m_meshes[i];
        numVertexes = static_cast<uint32_t>( source_mesh.positions.size() );
        numTriangles = static_cast<uint32_t>( source_mesh.triangles.size() );
        mesh.name = source_mesh.name;
        mesh.shader = source_mesh.shader;

        if( numVertexes != source_mesh.coordinates.size() || 
            numVertexes != source_mesh.weights.size() ||
            numVertexes != source_mesh.joints.size() )
            throw std::runtime_error( "Mismatch"); /// todo handle better here

        mesh.vertices.reserve( numVertexes );
        for ( uint32_t v = 0; v < numVertexes; v++)
        {
            MD5::Vertex_t vertex{};
            auto position = source_mesh.positions[v];
            auto coordinate = source_mesh.coordinates[v];
            auto weights = source_mesh.weights[v];
            auto joints = source_mesh.joints[v];
            uint32_t numWeigts = weights.size();
            
            vertex.uv = coordinate;
            vertex.startWeight = nextWeight;
            /// check 
            if( weights.size() != joints.size() )
                throw std::runtime_error( "TODO: name error" );

            for ( uint32_t w = 0; w < numWeigts; w++)
            {
                MD5::Weight_t newWeight{};
                auto weightBias = weights[w];
                auto weightjoint = joints[w];

                if( weightBias < 0.001f )
                    continue;

                /// translate vertex from model position to joint position
                // weight in the joint space 
                auto inverseBindMatrix = md5Model.Joints()[weightjoint].ComputeInverseBindPose();
                newWeight.pos = inverseBindMatrix * position;
                newWeight.bias = weightBias;
                newWeight.joint = weightjoint;
                mesh.weights.push_back( newWeight );
                nextWeight++;
            }

            vertex.weightCount = nextWeight - vertex.startWeight;
            mesh.vertices.push_back( vertex );
        }

        mesh.triangles.reserve( numTriangles );
        for ( uint32_t t = 0; t < numTriangles; t++)
        {
            MD5::Triangle_t triangle{};
            triangle.v0 = source_mesh.triangles[t].x;
            triangle.v1 = source_mesh.triangles[t].y;
            triangle.v2 = source_mesh.triangles[t].z;
            mesh.triangles.push_back( triangle );
        }
        
        md5Model.AddMesh( mesh );
    }
    
    return md5Model;
}

/*
======================================
gltfToMd5::LoadTriangles
======================================
*/
bool gltfToMd5::LoadTriangles( const gltf::Asset &in_assets, const gltf::Primitive &in_prim )
{
    // We specify GenerateMeshIndices, so we should always have indices
	if( !in_prim.indicesAccessor.has_value() ); 
        return false;

    // Índices
    auto& indexAcc = in_assets.accessors[*in_prim.indicesAccessor];
    
    // ensure we have enough space for all triangles,
    // even if the count is not a multiple of 3
    std::vector<uint16_t> indices;
    indices.resize( indexAcc.count ); 
                
    if ( indexAcc.componentType == gltf::ComponentType::UnsignedByte || indexAcc.componentType == gltf::ComponentType::UnsignedShort )
    {
        gltf::copyFromAccessor<std::uint16_t>( in_assets, indexAcc, indices.data());

        for (size_t i = 0; i < indexAcc.count; i += 3 )
        {
            glm::u32vec3 t{};
            t.x = indices[ i + 0 ];
            t.y = indices[ i + 1 ];
            t.z = indices[ i + 2 ];
            m_meshes[m_currentMesh].triangles.push_back(t);
        }
    }

    return true;
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
    m_meshes[m_currentMesh].positions.resize( positionAccessor.count );
    m_meshes[m_currentMesh].coordinates.resize( texCoordAccessor.count );

    // position data acess lambda
    auto getpos = [&]( fastgltf::math::fvec3 pos, std::size_t idx ) 
    {
		m_meshes[m_currentMesh].positions[idx] = glm::vec4( pos.x(), pos.y(), pos.z(), 1.0f );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec3>( in_assets, positionAccessor, getpos );
 
    // coordinate data acess lambda
    auto getuv = [&]( fastgltf::math::fvec2 uv, std::size_t idx ) 
    {
	    m_meshes[m_currentMesh].coordinates[idx] = glm::vec2( uv.x(), uv.y() );
    };

    fastgltf::iterateAccessorWithIndex<fastgltf::math::fvec2>( in_assets, texCoordAccessor, getuv );
    return true;
}

/*
======================================
gltfToMd5::LoadWeights
======================================
*/
bool gltfToMd5::LoadWeights( const gltf::Asset &in_assets, const gltf::Primitive& in_prim )
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
        m_meshes[m_currentMesh].weights.resize( weightAccessor0.count + weightAccessor1.count );
        m_jointPerVertex = 8;
    }
    else
    {
        m_meshes[m_currentMesh].weights.resize( weightAccessor0.count );
        m_jointPerVertex = 4;
    }

    // resize the joint array
    if( jointsAccessor1.bufferViewIndex.has_value() )
        m_meshes[m_currentMesh].joints.resize( jointsAccessor0.count + jointsAccessor1.count );
    else
        m_meshes[m_currentMesh].joints.resize( jointsAccessor0.count );

    // coordinate weight acess lambda
    auto Getweight = [&]( fastgltf::math::fvec4 weight, std::size_t idx ) 
    {   
        for (size_t w = 0; w < 4; w++)
        {
            m_meshes[m_currentMesh].weights[windex + w].push_back( weight[w] );
        }        
        windex += 4;
    };

    // coordinate joint acess lambda
    auto Getjoint = [&]( fastgltf::math::u16vec4 joint, std::size_t idx ) 
    {
        for (size_t wj = 0; wj < 4; wj++)
        {
            m_meshes[m_currentMesh].joints[jindex + wj].push_back( joint[wj] );
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
gltfToMd5::LoadJoints
======================================
*/
void gltfToMd5::LoadJoints( const gltf::Asset &in_assets, const gltf::Skin& in_skin )
{
    uint32_t i = 0;
    uint32_t jointCount = 0;
    auto nodes = in_assets.nodes;

    /// TODO: if no joint found, create a root joint

    jointCount = static_cast<uint32_t>( in_skin.joints.size() );
    m_joints.reserve( jointCount );

    // Descobre parent de cada node
    std::vector<int> parent( nodes.size(), -1 );
    for ( i = 0; i < nodes.size(); i++) 
    {
        for (auto c : nodes[i].children)
            parent[c] = static_cast<int>( i );
    }

    for ( i = 0; i < jointCount; i++) 
    {
        uint32_t nodeIndex = in_skin.joints[i];
        const gltf::Node& node = nodes[nodeIndex];

        joint_t joint;

        /// if no joint name, "create" one
        if ( node.name.empty() )
            joint.name =  "joint_" + std::to_string(i);
        else
            joint.name = node.name.c_str();

        joint.parent = parent[nodeIndex];
        
        auto transform = std::get<gltf::TRS>( node.transform );

        /// get translation
        joint.position = glm::vec4( transform.translation.x(), transform.translation.y(), transform.translation.z(), 1.0f );
 
        /// get rotation
        joint.rotation = glm::vec4( transform.rotation.w(), transform.rotation.x(), transform.rotation.y(), transform.rotation.z());
        
        /// get scaling ( not used in MD5Mesh 1.0 )
        joint.scalation = glm::vec4( transform.scale.x(), transform.scale.y(), transform.scale.z(), 1.0f );

        m_joints.push_back( joint );
    }
}

/*
======================================
gltfToMd5::LoadMesh
======================================
*/
void gltfToMd5::LoadMeshes( const gltf::Asset &in_assets, const gltf::Skin& in_skin )
{
    m_meshes.reserve( in_assets.meshes.size() );

    /// load model meshes
    for ( const auto& mesh : in_assets.meshes )
    {
        for ( auto& prim : mesh.primitives )
        {
            /// MD5meshes only suport triangles
            if ( prim.type != fastgltf::PrimitiveType::Triangles )
                    continue;

            size_t vertexCount = 0;
            uint32_t jointPerVertex = 0;
            std::vector<glm::vec4>      positions = std::vector<glm::vec4>();   // vertex positions ( vec4 for easy joit pos calculation )
            std::vector<glm::vec2>      coordinates = std::vector<glm::vec2>(); // vertex uv 
            std::vector<float>          weights = std::vector<float>();     // joints influences
            std::vector<uint16_t>       joints = std::vector<uint16_t>();   // joint ids
            std::vector<glm::mat4>      inverseBindMatrix = std::vector<glm::mat4>(); // inverse pose matrix
            
            if ( prim.materialIndex.has_value() )
            {
                const auto &mtr = in_assets.materials[*prim.materialIndex];
                m_meshes[m_currentMesh].shader = mtr.name.c_str();
            }
            else
            {
                m_meshes[m_currentMesh].shader = "defaut";
            }

            if( !LoadTriangles( in_assets, prim ) )
                throw std::runtime_error( "A mesh primitive is required to hold the indices." );

            // get vertex positions
            if( !LoadVertexes( in_assets, prim ) )
                throw std::runtime_error( "A mesh primitive is required to hold the POSITION attribute." );

            // get vertex weights influences
            if( !LoadWeights( in_assets, prim ) )
                throw std::runtime_error( "A mesh primitive is required to hold the WEIGHTS_0 attribute." );

            // inverse pose Bind Matrix
            if( !LoadInverseMatrixes( in_assets, in_skin, inverseBindMatrix ) )
                throw std::runtime_error( "A mesh primitive is required to hold the WEIGHTS_0 attribute." );

        m_currentMesh++;
        }
    }
}
