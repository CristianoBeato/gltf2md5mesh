
#include "md5Model.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iomanip>
#include <fstream>
#include <sstream>

void MD5::Joint_t::ComputeW(void)
{
    float t = 1.0f - ( orient.x * orient.x ) - (orient.y * orient.y) - (orient.z * orient.z);
    orient.w = t < 0.0f ? 0.0f : -std::sqrt( t );
}

void MD5::Joint_t::ComputeInverseBindPose(void)
{
    glm::mat4 transform = glm::translate( glm::mat4(1.0f), pos ) * glm::mat4_cast( orient );
    glm::mat4 inverseBind = glm::inverse( transform );
}

MD5::Model::Model(void)
{
}

MD5::Model::~Model(void)
{
}

void MD5::Model::Read( const std::string &in_path )
{
    uint32_t i = 0;
    uint32_t jointIndex = 0;
    uint32_t meshIndex = 0;
    std::string line;

    std::ifstream file( in_path );
    if ( !file.is_open() )
        throw std::runtime_error( "Erro: failed to read model" );

    while ( std::getline( file, line ) ) 
    {
        if (line.empty()) 
            continue;

        std::istringstream iss( line );

        std::string token;
        iss >> token;

        // Retrieve the version of the current model.
        if ( token == "MD5Version" )
        {
            uint32_t version;
            iss >> version;
            m_version = version;    /// TODO: may we check version here ?
        }
        // Retrieve the command line used to idTech4 engine tools.
        else if ( token == "commandline" )
        {
            std::string cmdline;
            std::getline( iss, cmdline );
            m_commandline = cmdline;
        }
        // Retrieve the joint count and reserve space for the joints array.
        else if( token == "numJoints" )
        {
            uint32_t numJoints;
            iss >> numJoints;
            m_joints.resize( numJoints );
        }
        // Retrieve the mesh count and reserve space for the meshes array.
        else if ( token == "numMeshes" )
        {
            uint32_t numMeshes;
            iss >> numMeshes;
            m_meshes.resize( numMeshes );
        }
        // Retrieve the joints block and fill the joints array.
        else if ( token == "joints" )
        {
            // read joints block
            while ( std::getline( file, line ) )
            {
                // end of joints block
                if ( line == "}" )
                    break;

                std::istringstream jointIss( line );
                std::string jointName;
                int parentIndex;
                float posX, posY, posZ;
                float orientX, orientY, orientZ;

                // Exemp: "bone1" 0 ( 10 0 0 ) ( 0.707 0 0.707 )
                jointIss >> std::quoted( jointName ) >> parentIndex;
                jointIss.ignore( 100, '(' ); // ignore until '('
                jointIss >> posX >> posY >> posZ;
                jointIss.ignore( 100, '(' ); // ignore until '('
                jointIss >> orientX >> orientY >> orientZ;

                Joint_t joint{};
                joint.name = jointName;
                joint.parentIndex = parentIndex;
                joint.pos = glm::vec3( posX, posY, posZ );
                joint.orient = glm::vec3( orientX, orientY, orientZ );
                joint.ComputeW();
                m_joints[jointIndex++] = joint;
            }
        }
        else if ( token == "mesh" )
        {
            Mesh_t &mesh = m_meshes[meshIndex];

            // read mesh block
            while ( std::getline( file, line ) )
            {
                if ( line == "}" )
                {
                    meshIndex++;
                    break;
                }

                std::istringstream meshIss( line );
                std::string meshToken;
                meshIss >> meshToken;

                /// Retrieve mesh material name.
                if ( meshToken == "shader" )
                {
                    std::string shaderName;
                    meshIss >> std::quoted( shaderName );
                    mesh.shader = shaderName;
                }
                /// Retrieve vertex count and vertex data.
                else if ( meshToken == "numverts" )
                {
                    uint32_t numVerts;
                    meshIss >> numVerts;
                    mesh.vertices.reserve( numVerts );
                    
                    /// Read vertex data lines.
                    for ( i = 0; i < numVerts; i++ )
                    {
                        std::getline( file, line );

                        // we don't found a valid vertex 
						if ( line.find("vert") == std::string::npos )
							break;

                        std::istringstream vertIss( line );
                        std::string vertToken;
                        vertIss >> vertToken; // should be "vert"

                        uint32_t vertIndex;
                        float uvX, uvY;
                        int startWeight, weightCount;

                        vertIss.ignore( 100, '(' ); // ignore until '('
                        vertIss >> uvX >> uvY;
                        vertIss.ignore( 100, '(' ); // ignore until '('
                        vertIss >> startWeight >> weightCount;

                        Vertex_t vertex{};
                        vertex.uv = glm::vec2( uvX, uvY );
                        vertex.startWeight = startWeight;
                        vertex.weightCount = weightCount;

                        mesh.vertices.push_back( vertex );
                    }
                }
                else if( meshToken == "numtris" )
                {
                    uint32_t numTris;
                    meshIss >> numTris;
                    mesh.triangles.reserve( numTris );

                    /// Read triangle data lines.
                    for ( i = 0; i < numTris; i++ )
                    {
                        std::getline( file, line );

                        // we don't found a valid triangle
                        if ( line.find("tri") == std::string::npos )
                            break;

                        std::istringstream triIss( line );
                        std::string triToken;
                        triIss >> triToken; // should be "tri"
                        Triangle_t triangle{};
                        uint32_t triIndex;
                        triIss >> triIndex >> triangle.v0 >> triangle.v1 >> triangle.v2;
                        mesh.triangles.push_back( triangle );
                    }
                }
                else if( meshToken == "numweights" )
                {
                    uint32_t numWeights;
                    meshIss >> numWeights;
                    mesh.weights.reserve( numWeights );

                    /// Read weight data lines.
                    for ( i = 0; i < numWeights; i++ )
                    {
                        std::getline( file, line );

                        // we don't found a valid weight
                        if ( line.find("weight") == std::string::npos )
                            break;

                        std::istringstream weightIss( line );
                        std::string weightToken;
                        weightIss >> weightToken; // should be "weight"

                        uint32_t weightIndex;

                        /// Get weight data: joint index, bias and position in joint space.
                        Weight_t weight{};
                        weightIss >> weightIndex >> weight.joint >> weight.bias;
                        weightIss.ignore( 100, '(' ); // ignore until '('
                        weightIss >> weight.pos.x >> weight.pos.y >> weight.pos.z;
                        mesh.weights.push_back( weight );
                    }
                }
            }
        }
    }
}

void MD5::Model::Write(const std::string &in_path) const
{
	uint32_t i = 0;
	uint32_t j = 0;
	std::ofstream file( in_path );
    if (!file.is_open()) 
        std::runtime_error( "Erro: failed to write model" ); 

    file << std::fixed;             // never in exponencial
    file << std::setprecision(6);   // 6 decimal digits (like Doom 3)

	// write file version
	file << "MD5Version " << m_version << "\n";

	// write commandline
	file << "commandline " << "\"" << m_commandline << "\"\n\n";

	// write the joints and meshes count
	file << "numJoints " << m_joints.size() << "\n";
	file << "numMeshes " << m_meshes.size() << "\n\n";

	file << "joints\n{\n";

	for ( i = 0; i < m_joints.size(); i++)
	{
		MD5::Joint_t joint = m_joints[i];
		
		// write joint "name" and parent index ( -1 for root )
		file << "\t\"" << joint.name << "\" " << joint.parentIndex;
		
		// write joint position
		file << " ( " << joint.pos.x << " " << joint.pos.y << " " << joint.pos.z << " ) ";
		
		// write joint orientation
		file << " ( " << joint.orient.x << " " << joint.orient.y << " " << joint.orient.z << " )\n";
	}

	file << "}\n";
	file << std::endl;

	for ( i = 0; i < m_meshes.size(); i++)
	{
		MD5::Mesh_t mesh = m_meshes[i];
        /// debug purpose, write the mesh name as a comment in the md5mesh file.
        file << "// Mesh " << i << ": " << mesh.name << std::endl;

        /// Begin mesh block
		file << "mesh\n{\n";

		file << "\tshader \"" << mesh.shader << "\"\n\n";

        /// Write out the vertices array.
		file << "\tnumverts " << mesh.vertices.size() << "\n";
		for ( j = 0; j < mesh.vertices.size(); j++ )
		{
			auto vert = mesh.vertices[j];
			file << "\tvert " << j << " ( " << vert.uv.x << " " << vert.uv.y << " ) " << vert.startWeight << " " << vert.weightCount << "\n";
		}
		file << std::endl;

        /// Write out the triangles array.
		file << "\tnumtris " << mesh.triangles.size() << "\n";
		for ( j = 0; j < mesh.triangles.size(); j++)
		{
			auto tri = mesh.triangles[j];
			file << "\ttri " << j << " " << tri.v0 << " " << tri.v1 << " " << tri.v2 << "\n";
		}
		file << std::endl;

        /// Write out the weights array.
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

void MD5::Model::Clear(void)
{
    uint32_t i = 0;
    for ( i = 0; i < m_meshes.size(); i++)
    {
        m_meshes[i].vertices.clear();
        m_meshes[i].triangles.clear();
        m_meshes[i].weights.clear();
    }

    m_meshes.clear();
    m_joints.clear();
}
