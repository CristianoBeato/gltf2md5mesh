
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

glm::mat4 MD5::Joint_t::ComputeInverseBindPose( void ) const
{
    glm::mat4 transform = glm::translate( glm::mat4(1.0f), pos ) * glm::mat4_cast( orient );
    return glm::inverse( transform );
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

    std::cout << "Reading md5mesh file: " << in_path << std::endl;

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
        else if (token == "joints") 
        {
            while ( std::getline( file, line ) && line.find("}") == std::string::npos ) 
            {
                std::istringstream jointIss(line);
                std::string jointName;
                if ( !( jointIss >> std::quoted( jointName ) ) ) 
                    continue; // jump '{' line

                uint32_t parentIndex;
                jointIss >> parentIndex;
                
                Joint_t joint{};
                joint.name = jointName;
                joint.parentIndex = parentIndex;

                jointIss.ignore( 256, '(');
                jointIss >> joint.pos.x >> joint.pos.y >> joint.pos.z;
                jointIss.ignore( 256, ')');
                jointIss >> joint.orient.x >> joint.orient.y >> joint.orient.z;
                
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
                if ( line.find("}") != std::string::npos )
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
                else if (meshToken == "numverts") 
                {
                    uint32_t numVerts;
                    meshIss >> numVerts;
                    mesh.vertices.resize( numVerts );

                    for ( i = 0; i < numVerts; ++i ) 
                    {
                        if ( !std::getline( file, line ) ) 
                            break;

                        // If the line is empty or is just a comment, skip it.
                        if ( line.empty() || line.find( "vert" ) == std::string::npos ) 
                        {
                            i--; // Skip empty lines or comments.
                            if (file.eof()) 
                                break;

                            continue;
                        }

                        std::istringstream vertIss(line);
                        std::string junk;
                        uint32_t vertIndex;
                        Vertex_t vertex{};

                        // vert [index] ( [u] [v] ) [startWeight] [countWeight]
                        vertIss >> junk >> vertIndex; // consumes "vert" and the index

                        vertIss.ignore( 256, '(' );
                        vertIss >> vertex.uv.x >> vertex.uv.y;

                        vertIss.ignore( 256, ')' ); 
                        vertIss >> vertex.startWeight >> vertex.weightCount;

                        mesh.vertices[vertIndex] = vertex;
                    }
                }
                else if( meshToken == "numtris" )
                {
                    uint32_t numTris;
                    meshIss >> numTris;
                    mesh.triangles.resize( numTris );

                    /// Read triangle data lines.
                    for ( i = 0; i < numTris; i++ )
                    {
                        std::getline( file, line );

                        // If the line is empty or is just a comment, skip it.
                        if ( line.empty() || line.find( "tri" ) == std::string::npos ) 
                        {
                            i--; // Skip empty lines or comments.
                            if (file.eof()) 
                                break;

                            continue;
                        }

                        std::istringstream triIss( line );
                        std::string triToken;
                        triIss >> triToken; // should be "tri"
                        Triangle_t triangle{};
                        uint32_t triIndex;
                        triIss >> triIndex >> triangle.v0 >> triangle.v1 >> triangle.v2;
                        mesh.triangles[triIndex] = triangle;
                    }
                }
                else if ( meshToken == "numweights" ) 
                {
                    uint32_t numWeights;
                    if (! ( meshIss >> numWeights ) ) 
                        continue;

                    mesh.weights.resize( numWeights );

                    for ( i = 0; i < numWeights; ++i) 
                    {
                        if (!std::getline(file, line))
                            break;

                        std::istringstream weightIss( line );
                        std::string token;
                        weightIss >> token;

                        if (token != "weight") 
                        {
                            i--; // Skip empty lines or comments.
                            if (file.eof()) 
                                break;

                            continue;
                        }

                        uint32_t weightIndex;
                        Weight_t weight{};

                        // Reading: weight [index] [joint] [bias]
                        weightIss >> weightIndex >> weight.joint >> weight.bias;

                        // Reading: ( [x] [y] [z] )
                        weightIss.ignore( 256, '(' );
                        weightIss >> weight.pos.x >> weight.pos.y >> weight.pos.z;
                        weightIss.ignore( 256, ')' ); 

                        // set weight at index 
                        mesh.weights[weightIndex] = weight;
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

bool MD5::Model::ValidateModel( std::stringstream &out_error ) const
{
    bool success = true;
    uint32_t base = 0;
    uint32_t i = 0;
    uint32_t jointCount = m_joints.size();
    uint32_t meshCount = m_meshes.size();

    for ( i = 0; i < jointCount; i++)
    {
        auto joint = m_joints[i];
        if ( joint.parentIndex < 0 )
        {
            if( base == UINT32_MAX )
            {
                base = i;
            }
            else
            {
                out_error << "Joint " << i << " " << joint.name  << " has no parent, but joint " << base << " already is base\n";
                // actually is not a error have two or more root joint
            }
            base = i;
        }
        
    }

    for( i = 0; i < meshCount; i++ )
    {
        auto mesh = m_meshes[i];
        out_error << "Mesh " << i << "\n";
        if( !mesh.ValidateMesh( jointCount, out_error ) )
            success = false;
    }    

    return success;
}

bool MD5::Mesh_t::ValidateMesh( const uint32_t in_numJoints, std::stringstream &out_error ) const
{
    bool success = true;
    uint32_t i = 0, j = 0;
    uint32_t triangleCount = triangles.size();
    uint32_t vertexCount = vertices.size();
    uint32_t weightCount = weights.size();

    /// Todo check if triangles don't are out of vertex bounds 
    for ( i = 0; i < triangleCount; i++)
    {
        auto triangle = triangles[i];
        for ( j = 0; j < 3; j++)
        {
            uint32_t v = triangle[j];
            
            /// Check if 
            if( v >= vertexCount )
            {
                out_error << "\ttriangle nº " << i << " vertice " << j << " out of vertex array bounds " << vertexCount << "\n";
                success = false;
            }
        }
    }

    for ( i = 0; i < vertexCount; i++)
    {
        auto vertex = vertices[i];
        if( ( vertex.startWeight + vertex.weightCount ) >= weightCount )
        {
            out_error << "\tvertex nº " << i << " weigth start at (" << vertex.startWeight;
            out_error << ") and end at (" << vertex.startWeight + vertex.weightCount << ") is out of weights bound " << weightCount << "\n";
            success = false;
        }
    }

    for ( i = 0; i < weightCount; i++)
    {
        auto weight = weights[i];
        if ( weight.joint >= in_numJoints )
        {
            out_error << "\tWeight nº" << i << " joint is " << weight.joint << " maximum is " << weightCount << "\n";
            success = false;
        }
    }

    return success;
}
