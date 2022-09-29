#pragma once

#include "collider_shape.h"
#include "ALP__meshcoll_data.h"

#include <unordered_map>

namespace Adollib {

	// Meshcoll_partは隠す
	namespace Physics_function {
		//Mesh colliderクラス
		class Meshcoll_part : public Collider_shape {
		public:
			Vector3 center;
			Vector3 rotate;
			Vector3 size;

		private:

		public:
			//不動オブジェクトとして生成
			Meshcoll_part(Physics_function::ALP_Collider* l_ALPcollider_ptr)
				:center(Vector3(0)), rotate(Vector3(0)), size(1) {
				shape_tag = Physics_function::ALPCollider_shape_type::Mesh;
				ALPcollider_ptr = l_ALPcollider_ptr;
			};

			void adapt_Colliderdata() override {
				local_position = center;
				local_orientation = quaternion_from_euler(rotate);
				local_scale = size;
			};

			void initiazlie(Physics_function::Shape_InitData* base) override {
				center = base->center;
				rotate = base->rotate;
				size = base->size;
			}

			void update_dop14() override {
				if (mesh_data == nullptr) {
					for (int i = 0; i < DOP::DOP_size; i++) {
						dop14.max[i] = 0;
						dop14.min[i] = 0;
					}
					return;
				}

				dop14.pos = world_position();

				//各軸のmin,maxのリセット
				Vector3 rotated_axis[DOP::DOP_size];
				for (int i = 0; i < DOP::DOP_size; i++) {
					rotated_axis[i] = vector3_quatrotate(DOP::DOP_14_axis[i], world_orientation().inverse()).unit_vect();
					dop14.max[i] = -FLT_MAX;
					dop14.min[i] = +FLT_MAX;
				}

				for (int v_num = 0; v_num < 8; v_num++) {
					const Vector3& pos = mesh_data->base_pos[v_num] * world_scale();

					//DOPの更新
					for (int i = 0; i < DOP::DOP_size; i++) {
						const float dis = vector3_dot(rotated_axis[i], pos);
						if (dop14.min[i] > dis) dop14.min[i] = dis * 1.00000001f;//確実にするためちょっと大きめにとる
						if (dop14.max[i] < dis) dop14.max[i] = dis * 1.00000001f;//確実にするためちょっと大きめにとる

					}

				}
			};

			const Matrix33 local_tensor() const override {
				const Vector3& Wsize = world_scale();
				Matrix33 ret;

				ret = matrix33_identity();
				ret._11 = 0.3333333f * ((Wsize.y * Wsize.y) + (Wsize.z * Wsize.z));
				ret._22 = 0.3333333f * ((Wsize.z * Wsize.z) + (Wsize.x * Wsize.x));
				ret._33 = 0.3333333f * ((Wsize.x * Wsize.x) + (Wsize.y * Wsize.y));
				return ret;
			};

			const float get_volume() const override {
				return local_scale.x * local_scale.y * local_scale.z;
			}

			void load(const std::vector<int>& l_indexes, const std::vector<Vector3>& l_vertices, bool is_RightTriangle = false) {

				if (mesh_data != nullptr)delete mesh_data;
				mesh_data = new Meshcollider_data();

				auto& index_num = mesh_data->index_num;
				auto& vertex_num = mesh_data->vertex_num;
				auto& facet_num = mesh_data->facet_num;
				auto& edge_num = mesh_data->edge_num;

				auto& indexes = mesh_data->indexes;
				auto& vertices = mesh_data->vertices;
				auto& facets = mesh_data->facets;
				auto& edges = mesh_data->edges;
				auto& vertex_involvements = mesh_data->vertex_involvements; //頂点番号から該当の面にアクセスするためのもの

				auto& dopbase = mesh_data->dopbase;

				auto& is_Convex = mesh_data->is_Convex;

				index_num = l_indexes.size();
				vertex_num = l_vertices.size();
				indexes = l_indexes;
				vertices = l_vertices;
				vertex_involvements.resize(vertex_num);
				is_Convex = true;

				//面情報の保存
				{
					facet_num = index_num / 3;
					Physics_function::Facet F;


					for (u_int i = 0; i < facet_num; i++) {
						const int indexed[3] = {
							indexes.at(i * 3 + 0),
							indexes.at(i * 3 + 1),
							indexes.at(i * 3 + 2)
						};

						const Vector3 normal = vector3_cross(vertices[indexed[1]] - vertices[indexed[0]], vertices[indexed[2]] - vertices[indexed[0]]);

						// 面の面積がとても小さければ面に追加しない
						if (normal.norm() < FLT_EPSILON) {
							continue;
						}

						if (is_RightTriangle) {
							F.vertexID[0] = indexed[0];
							F.vertexID[1] = indexed[1];
							F.vertexID[2] = indexed[2];
							F.normal = +normal;
							F.normal = F.normal.unit_vect();
						}
						else {
							F.vertexID[2] = indexed[0];
							F.vertexID[1] = indexed[1];
							F.vertexID[0] = indexed[2];
							F.normal = -normal;
							F.normal = F.normal.unit_vect();
						}

						// vertexからfacetへアクセスできるように保存
						vertex_involvements.at(indexed[0]).add_facet_involvment(facets.size());
						vertex_involvements.at(indexed[1]).add_facet_involvment(facets.size());
						vertex_involvements.at(indexed[2]).add_facet_involvment(facets.size());

						facets.emplace_back(F);
					}

					facet_num = facets.size();
				}

				//エッジ情報の保存
				{
					edge_num = 0;
					std::unordered_map<int, int> edgeID_Table;

					Physics_function::Edge E;
					for (u_int i = 0; i < facet_num; i++) {
						Physics_function::Facet& facet = facets[i];

						// 面の中で一番長いedgeを求める
						u_int longer_edge_id = 0;
						{
							float max_length = 0;
							for (int o = 0; o < 3; o++) {

								const u_int vertId0 = ALmin(facet.vertexID[o % 3], facet.vertexID[(o + 1) % 3]);
								const u_int vertId1 = ALmax(facet.vertexID[o % 3], facet.vertexID[(o + 1) % 3]);

								if (max_length < vector3_distance(vertices[vertId0], vertices[vertId1])) {
									max_length = vector3_distance(vertices[vertId0], vertices[vertId1]);
									longer_edge_id = vertId0 + vertId1;
								};

							}
						}

						// edge情報を保存
						for (int o = 0; o < 3; o++) {
							const u_int vertId0 = ALmin(facet.vertexID[o % 3], facet.vertexID[(o + 1) % 3]);
							const u_int vertId1 = ALmax(facet.vertexID[o % 3], facet.vertexID[(o + 1) % 3]);

							const u_int b = vertId1 * vertId1 - vertId1;

							////int tableId = (vertId1 * vertId1 - vertId1) * 0.5f + vertId0;
							//float tableId_ = (intId1 * (intId1 - 1)) * 0.5f + intId0;

							// vertId0, vertId1から作られるユニークな数字
							int tableId = (int)(b + vertId0 * 2);

							if (edgeID_Table.count(tableId) == 0) {
								// 初回時は登録のみ
								E.facetID[0] = i;
								E.facetID[1] = i;
								E.vertexID[0] = vertId0;
								E.vertexID[1] = vertId1;
								E.type = Physics_function::Edgetype::EdgeConvex; // 凸エッジで初期化

								// vertexからedgeへアクセスできるように保存
								bool is_longer = false;
								if (longer_edge_id == vertId0 + vertId1)is_longer = true;
								vertex_involvements.at(vertId0).add_edge_involvment(edges.size(), is_longer);
								vertex_involvements.at(vertId1).add_edge_involvment(edges.size(), is_longer);

								edges.emplace_back(E);

								facet.edgeID[o] = edge_num;
								edgeID_Table[tableId] = edge_num;

								edge_num++;
							}
							else {
								// すでに登録されていた
								Physics_function::Edge& edge = edges[edgeID_Table[tableId]];
								Physics_function::Facet& facetB = facets[edge.facetID[0]];

								if (edge.facetID[0] != edge.facetID[1]) {
									int dafsgdhf = 0;

									auto& a = facets[edge.facetID[0]];
									auto& b = facets[edge.facetID[1]];
									int s = 0;
								}
								//assert(edge.facetID[0] == edge.facetID[1] && "Don't let the edges have 3～ facet.");


								// エッジに含まれないＡ面の頂点がB面の表か裏かで判断
								Vector3 s = vertices[facet.vertexID[(o + 2) % 3]];
								Vector3 q = vertices[facetB.vertexID[0]];
								float d = vector3_dot(s - q, facetB.normal);

								//エッジの種類を入力
								if (d < -FLT_EPSILON) {
									edge.type = Physics_function::Edgetype::EdgeConvex;
								}
								else if (d > FLT_EPSILON) {
									edge.type = Physics_function::Edgetype::EdgeConcave;
									is_Convex = false;
								}
								else {
									edge.type = Physics_function::Edgetype::EdgeFlat;
								}

								edge.facetID[1] = i;

								facet.edgeID[o] = edgeID_Table[tableId];
							}

						}


					}
				}

				// DOPの計算
				{
					for (int i = 0; i < DOP::DOP_size; i++) {
						dopbase.max[i] = -FLT_MAX;
						dopbase.min[i] = +FLT_MAX;
					}

					//初期状態のDOPの保存 loop中に計算するDOPはこのDOPを基準にする
					for (int i = 0; i < DOP::DOP_size; i++) {
						for (int& ind : indexes) {

							float dis = vector3_dot(DOP::DOP_14_axis[i], vertices.at(ind));
							dopbase.max[i] = ALmax(dopbase.max[i], dis);
							dopbase.min[i] = ALmin(dopbase.min[i], dis);
						}
					}

					{
						for (int x_ = 0; x_ < 2; x_++) {
							float x_dis = (x_ == 0 ? dopbase.max[0] : dopbase.min[0]);

							for (int y_ = 0; y_ < 2; y_++) {
								float y_dis = (y_ == 0 ? dopbase.max[1] : dopbase.min[1]);

								for (int z_ = 0; z_ < 2; z_++) {
									float z_dis = (z_ == 0 ? dopbase.max[2] : dopbase.min[2]);

									Crossing_func::getCrossingP_three_plane(
										DOP::DOP_14_axis[0], x_dis,
										DOP::DOP_14_axis[1], y_dis,
										DOP::DOP_14_axis[2], z_dis,
										mesh_data->base_pos[z_ + y_ * 2 + x_ * 4]
									);

								}
							}
						}
					}


					update_dop14();
				}


			}





		};


	}




}
