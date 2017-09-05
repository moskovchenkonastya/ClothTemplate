#include "ClothSim.h"
#include <cstdint>
#include <cmath>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ClothMeshData CreateTest2Vertices()
{
  ClothMeshData mdata;
  // заполн€ю сетку 15 * 20
  for(int i = 0; i < 15; i++){
	  for(int j = 0; j < 20; j++){
		  mdata.vertPos0.push_back(float4(0.1 * i - 0.75, 0.1 * j - 0.5, 0, 1));// координаты точек
		  mdata.vertVel0.push_back(float4(0, 0, 0, 0));// cскорость
		  mdata.vertMassInv.push_back(1);//  масса точек
		  mdata.vertForces.push_back (float4(0, 0, 0, 0));// силы
	  }
  }

  mdata.vertPos1 = mdata.vertPos0;
  mdata.vertVel1 = mdata.vertVel0;

  float edgeLength = 0.1;
  float initialHardnes = 10;// жесткость к

  mdata.g_wind = float4(0, 0, 0, 0);
  //  код из пдфки задани€, который соедин€ет смежные вершины пружинами
  for (size_t i = 0; i < mdata.vertPos0.size(); i++) 
  { 
	  for (size_t j = i + 1; j < mdata.vertPos0.size(); j++) 
	  { 
		  float4 vA = mdata.vertPos0[i]; 
		  float4 vB = mdata.vertPos0[j]; 
		  float dist = length3(vB-vA); //sqrtf((vA.x - vB.x) * (vA.x - vB.x) + (vA.y - vB.y) * (vA.y - vB.y) + (vA.z - vB.z) * (vA.z - vB.z));
		  if (dist < 2.0f * sqrtf(2.0f) * edgeLength) 
		  { 
			  float hardness = initialHardnes * (edgeLength / dist); 
			  mdata.edgeIndices.push_back(i); 
			  mdata.edgeIndices.push_back(j); 
			  mdata.edgeHardness.push_back(hardness); 
			  mdata.edgeInitialLen.push_back(dist);
		  }
	  }
  }

  // you can use any intermediate mesh representation or load data to GPU (in VBOs) here immediately.                              <<===== !!!!!!!!!!!!!!!!!!

  // create graphics mesh; SimpleMesh uses GLUS Shape to store geometry; 
  // we copy data to GLUS Shape, and then these data will be copyed later from GLUS shape to GPU 
  //
  mdata.pMesh = std::make_shared<SimpleMesh>();

  GLUSshape& shape = mdata.pMesh->m_glusShape;

  shape.numberVertices = mdata.vertPos0.size();
  shape.numberIndices  = mdata.edgeIndices.size();

  shape.vertices  = (GLUSfloat*)malloc(4 * shape.numberVertices * sizeof(GLUSfloat));
  shape.indices   = (GLUSuint*) malloc(shape.numberIndices * sizeof(GLUSuint));

  memcpy(shape.vertices, &mdata.vertPos0[0], sizeof(float) * 4 * shape.numberVertices);
  memcpy(shape.indices, &mdata.edgeIndices[0], sizeof(int) * shape.numberIndices);

  // for tri mesh you will need normals, texCoords and different indices
  // 

  return mdata;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ClothMeshData::updatePositionsGPU()
{
	if (pMesh == nullptr)
		return;
	std::vector<float4>& inVertPos = (pinPong) ? vertPos1 : vertPos0;
	// copy current vertex positions to positions VBO

	glBindBuffer(GL_ARRAY_BUFFER, pMesh->m_vertexPosBufferObject); 
	CHECK_GL_ERRORS;
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNumber() * 4 * sizeof(GLfloat), (GLfloat*)&inVertPos[0]); 
	CHECK_GL_ERRORS;
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ClothMeshData::updateNormalsGPU()
{
  if (pMesh == nullptr || this->vertNormals.size() == 0)
    return;

  // copy current recalculated normals to appropriate VBO on GPU

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SimStep(ClothMeshData* pMesh, float delta_t)
{
 
  // get in and out pointers
  //
	float4* inVertPos  = pMesh->pinPong ? &pMesh->vertPos1[0] : &pMesh->vertPos0[0];
	float4* inVertVel  = pMesh->pinPong ? &pMesh->vertVel1[0] : &pMesh->vertVel0[0];

	float4* outVertPos = pMesh->pinPong ? &pMesh->vertPos0[0] : &pMesh->vertPos1[0];
	float4* outVertVel = pMesh->pinPong ? &pMesh->vertVel0[0] : &pMesh->vertVel1[0];

	// accumulate forces first
	//
	for (size_t i = 0; i < pMesh->vertForces.size(); i++) // clear all forces
		pMesh->vertForces[i] = float4(0, 0, 0, 0);

	//for (size_t i = 0; i < pMesh->vertexNumber(); i++)
		//outVertPos[i].y = inVertPos[i].y - 0.1;

	// цикл по св€з€м
	 for (int connectId = 0; connectId < pMesh->connectionNumber(); connectId++)
	{	
		
		int idx_A = 2 * connectId;
		int idx_B = 2 * connectId + 1;
		int alpha = pMesh->edgeIndices[idx_A];
		int beta = pMesh->edgeIndices[idx_B];
		float4 p1 = inVertPos[alpha];
		float4 p2 = inVertPos[beta]; 
		float dx = length3(p2-p1) - pMesh->edgeInitialLen[connectId];
		//  условие дл€ верхних точек
		if(pMesh->edgeIndices[2 * connectId] % 20 != 19){
			float4 F_elastic = -normalize(p2-p1) * pMesh->edgeHardness[connectId] * dx;
		//if(pMesh->edgeIndices[2 * connectId] % 20 != 19){
			pMesh->vertForces[alpha] += F_elastic;
			pMesh->vertForces[beta] -= F_elastic;
		}
 	}

	float maximum = 0;
	for (size_t i = 0; i < pMesh->vertPos0.size(); i++)
	{	
		if(inVertPos[i].z > maximum)
			maximum = inVertPos[i].z;
	}

	// update positions and velocity
	for (size_t i = 0; i < pMesh->vertPos0.size(); i++)
	{	
		
		if(i % 20 != 19){
			//  a = Fупр * 1/m + g + Fveter/m
			float4 a = pMesh->vertForces[i] * pMesh->vertMassInv[i] + float4(0, -9.8, 0, 0) + pMesh->g_wind * pMesh->vertMassInv[i];// - 0.25 * inVertVel[i] * pMesh->vertMassInv[i];
			float4 v = inVertVel[i] + a * delta_t;
			float4 pos = (v + inVertVel[i]) * delta_t + a * pow(delta_t, 2) * 0.5;
			outVertPos[i] = inVertPos[i] + pos;
			outVertVel[i] = inVertVel[i] + v;
			//outVertPos[i].w = 1;
		}
	}

	// for (size_t i = 0; i < pMesh->vertForces.size(); i++)
	//	outVertPos[i].y -= 0.01;

  
    pMesh->pinPong = !pMesh->pinPong; // swap pointers for next sim step
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RecalculateNormals(ClothMeshData* pMesh)
{

}

