#include <cstring>
#include <iostream>
#include <ostream>
#include <thread>

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

// JPH_SUPPRESS_WARNINGS

// ============= The test data and constants (using Z-up orientation!)

float cast(uint32_t u) {
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

// The time delta and horizontal linear velocity. The magnitude of the bad
// CollideShapeResult::mPenetrationDepth is extremely sensitive to both. This
// combination produces an especially large jump of ~4 meters, but single-frame
// jumps of at least 0.15 meters occur with almost any similar time delta and
// linear velocity values.
float test_delta_time() {
  return cast(0x3c880fb9);  // ~ 1/60 sec
}
JPH::Vec3 test_linear_velocity_xy() {
  return {cast(0xc0a53844), cast(0x40439dba), 0.0f};  // (-5.16312, 3.0565, 0.0)
}

JPH::Vec3 test_physics_system_gravity() {
  return {0.0f, 0.0f, -9.81f};
}

JPH::Vec3 test_character_position_initial() {
  // Standing in the corner seen in the video
  return {cast(0x42162bdb), cast(0x4211bda0), cast(0x3fe66666)};
}

JPH::Ref<JPH::CharacterVirtualSettings> test_character_virtual_settings() {
  static constexpr float kViewHeightPlayer = 1.8f;
  static constexpr float kShapeRadiusPlayer = 0.5f;
  static constexpr float kShapeTotalHeightPlayer =
      kViewHeightPlayer + kShapeRadiusPlayer;
  static constexpr float kShapeCylinderHalfHeightPlayer =
      (kShapeTotalHeightPlayer - 2.0f * kShapeRadiusPlayer) / 2.0f;

  // Create Z-oriented capsule shape with bottom at (0, 0, 0)
  const JPH::Vec3 translate_capsule{
      0.0f, 0.0f, kShapeCylinderHalfHeightPlayer + kShapeRadiusPlayer};
  const auto rotate_capsule =
      JPH::Quat::sRotation(JPH::Vec3::sAxisX(), M_PI * 0.5f);
  JPH::RefConst<JPH::Shape> shape_player =
      JPH::RotatedTranslatedShapeSettings(
          translate_capsule, rotate_capsule,
          new JPH::CapsuleShapeSettings(kShapeCylinderHalfHeightPlayer,
                                        kShapeRadiusPlayer))
          .Create()
          .Get();

  // Create CharacterVirtualSettings from capsule
  JPH::Ref<JPH::CharacterVirtualSettings> settings_cv =
      new JPH::CharacterVirtualSettings();
  settings_cv->mUp = JPH::Vec3::sAxisZ();
  // (accept support contacts on lower half-sphere of capsule)
  settings_cv->mSupportingVolume =
      JPH::Plane{JPH::Vec3::sAxisZ(), -0.9f * kShapeRadiusPlayer};
  settings_cv->mMaxSlopeAngle = JPH::DegreesToRadians(50.0f);
  settings_cv->mShape = shape_player;
  settings_cv->mPenetrationRecoverySpeed = 1.0f;

  return settings_cv;
}

void test_character_set_linear_velocity(
    JPH::CharacterVirtual* character_virtual, float delta_time) {
  // Move towards -x, +y with air control, jumping constantly
  JPH::Vec3 linear_velocity{cast(0xc0a53844), cast(0x40439dba), 0.0f};
  if (character_virtual->GetGroundState() ==
          JPH::CharacterVirtual::EGroundState::OnGround) {
    linear_velocity.SetZ(5.0f);
  } else {
    linear_velocity.SetZ(character_virtual->GetLinearVelocity().GetZ());
  }
  const JPH::Vec3 kVecJumpGravity{0.0f, 0.0f, -14.0f};
  linear_velocity += kVecJumpGravity * delta_time;

  character_virtual->SetLinearVelocity(linear_velocity);
}

JPH::CharacterVirtual::ExtendedUpdateSettings test_extended_update_settings() {
  // Bug occurs without stick-to-floor or walk-stairs, but I'm continuing to use
  // ExtendedUpdate to match my application

  JPH::CharacterVirtual::ExtendedUpdateSettings settings_eu{};
  settings_eu.mStickToFloorStepDown =
      JPH::Vec3{0.0f, 0.0f, 0.0f /* -kShapeRadiusPlayer */};
  settings_eu.mWalkStairsStepUp =
      JPH::Vec3{0.0f, 0.0f, 0.0f /* 0.8f * kShapeRadiusPlayer */};
  settings_eu.mWalkStairsStepDownExtra = JPH::Vec3{0.0f, 0.0f, 0.0f};

  return settings_eu;
}

std::vector<JPH::Ref<JPH::MeshShapeSettings>> test_vec_mesh_shape_settings() {
  std::vector<JPH::Ref<JPH::MeshShapeSettings>> vec;

  const JPH::uint32 kIndexMaterial = 0;

  // ground
  {
    JPH::VertexList list_vertex = {
        JPH::Float3{cast(0x42b40000), cast(0xc2200000), cast(0x0)},
        JPH::Float3{cast(0x42b40000), cast(0x42200000), cast(0x0)},
        JPH::Float3{cast(0xc1dccccc), cast(0xc2200000), cast(0x0)},
        JPH::Float3{cast(0xc1dccccc), cast(0x42200000), cast(0x0)},
    };
    JPH::IndexedTriangleList list_indexed_triangle = {
        JPH::IndexedTriangle{0, 1, 3, kIndexMaterial},
        JPH::IndexedTriangle{0, 3, 2, kIndexMaterial},
    };
    vec.emplace_back(new JPH::MeshShapeSettings(std::move(list_vertex),
                     std::move(list_indexed_triangle)));
  }
  // outer_corner_column.001
  {
    JPH::VertexList list_vertex = {
        JPH::Float3{cast(0x420d83dd), cast(0x4207f804), cast(0xbd1fff7a)},
        JPH::Float3{cast(0x420d83dd), cast(0x4207f804), cast(0x419fb000)},
        JPH::Float3{cast(0x421135ea), cast(0x42097fe2), cast(0xbd1fff7a)},
        JPH::Float3{cast(0x421135ea), cast(0x42097fe2), cast(0x419fb000)},
        JPH::Float3{cast(0x421445a7), cast(0x4210e3fc), cast(0xbd1fff7a)},
        JPH::Float3{cast(0x421445a7), cast(0x4210e3fc), cast(0x419fb000)},
        JPH::Float3{cast(0x42061fc2), cast(0x420b07c0), cast(0xbd1fff7a)},
        JPH::Float3{cast(0x42061fc2), cast(0x420b07c0), cast(0x419fb000)},
        JPH::Float3{cast(0x4212bdc9), cast(0x4214960a), cast(0xbd1fff7a)},
        JPH::Float3{cast(0x4212bdc9), cast(0x4214960a), cast(0x419fb000)},
        JPH::Float3{cast(0x420497e4), cast(0x420eb9ce), cast(0xbd1fff7a)},
        JPH::Float3{cast(0x420497e4), cast(0x420eb9ce), cast(0x419fb000)},
        JPH::Float3{cast(0x42074589), cast(0x42170ab7), cast(0x4122510c)},
        JPH::Float3{cast(0x4209145b), cast(0x4212ad5f), cast(0x40ec62bd)},
        JPH::Float3{cast(0x420cc668), cast(0x4214353d), cast(0x40ec62bd)},
        JPH::Float3{cast(0x420af797), cast(0x42189295), cast(0x4122510b)},
        JPH::Float3{cast(0x4211be6c), cast(0x41fb84fc), cast(0x41513143)},
        JPH::Float3{cast(0x42135e71), cast(0x41f3ac41), cast(0x41244c68)},
        JPH::Float3{cast(0x4217107e), cast(0x41f6bbfd), cast(0x41244c68)},
        JPH::Float3{cast(0x42157079), cast(0x41fe94b8), cast(0x41513143)},
        JPH::Float3{cast(0x421c3b04), cast(0x41c8e29e), cast(0x41755e6c)},
        JPH::Float3{cast(0x421d7494), cast(0x41c2f897), cast(0x41471b97)},
        JPH::Float3{cast(0x422126a2), cast(0x41c60853), cast(0x41471b97)},
        JPH::Float3{cast(0x421fed11), cast(0x41cbf25a), cast(0x41755e6c)},
        JPH::Float3{cast(0x4226b60e), cast(0x419647c2), cast(0x418700f1)},
        JPH::Float3{cast(0x4227a078), cast(0x4191dbe8), cast(0x415ef8a5)},
        JPH::Float3{cast(0x422b5285), cast(0x4194eba4), cast(0x415ef8a5)},
        JPH::Float3{cast(0x422a681c), cast(0x4199577e), cast(0x418700f1)},
        JPH::Float3{cast(0x42312d64), cast(0x41477d8a), cast(0x4190f94e)},
        JPH::Float3{cast(0x42320032), cast(0x413f89dc), cast(0x4172b9c4)},
        JPH::Float3{cast(0x4235b23f), cast(0x4145a956), cast(0x4172b9c4)},
        JPH::Float3{cast(0x4234df72), cast(0x414d9d04), cast(0x4190f94e)},
    };
    JPH::IndexedTriangleList list_indexed_triangle = {
        JPH::IndexedTriangle{2, 3, 1, kIndexMaterial},
        JPH::IndexedTriangle{2, 1, 0, kIndexMaterial},
        JPH::IndexedTriangle{3, 2, 4, kIndexMaterial},
        JPH::IndexedTriangle{3, 4, 5, kIndexMaterial},
        JPH::IndexedTriangle{0, 1, 7, kIndexMaterial},
        JPH::IndexedTriangle{0, 7, 6, kIndexMaterial},
        JPH::IndexedTriangle{6, 7, 11, kIndexMaterial},
        JPH::IndexedTriangle{6, 11, 10, kIndexMaterial},
        JPH::IndexedTriangle{5, 4, 8, kIndexMaterial},
        JPH::IndexedTriangle{5, 8, 9, kIndexMaterial},
        JPH::IndexedTriangle{13, 17, 16, kIndexMaterial},
        JPH::IndexedTriangle{13, 16, 12, kIndexMaterial},
        JPH::IndexedTriangle{14, 18, 17, kIndexMaterial},
        JPH::IndexedTriangle{14, 17, 13, kIndexMaterial},
        JPH::IndexedTriangle{15, 19, 18, kIndexMaterial},
        JPH::IndexedTriangle{15, 18, 14, kIndexMaterial},
        JPH::IndexedTriangle{17, 21, 20, kIndexMaterial},
        JPH::IndexedTriangle{17, 20, 16, kIndexMaterial},
        JPH::IndexedTriangle{18, 22, 21, kIndexMaterial},
        JPH::IndexedTriangle{18, 21, 17, kIndexMaterial},
        JPH::IndexedTriangle{19, 23, 22, kIndexMaterial},
        JPH::IndexedTriangle{19, 22, 18, kIndexMaterial},
        JPH::IndexedTriangle{21, 25, 24, kIndexMaterial},
        JPH::IndexedTriangle{21, 24, 20, kIndexMaterial},
        JPH::IndexedTriangle{22, 26, 25, kIndexMaterial},
        JPH::IndexedTriangle{22, 25, 21, kIndexMaterial},
        JPH::IndexedTriangle{23, 27, 26, kIndexMaterial},
        JPH::IndexedTriangle{23, 26, 22, kIndexMaterial},
        JPH::IndexedTriangle{25, 29, 28, kIndexMaterial},
        JPH::IndexedTriangle{25, 28, 24, kIndexMaterial},
        JPH::IndexedTriangle{26, 30, 29, kIndexMaterial},
        JPH::IndexedTriangle{26, 29, 25, kIndexMaterial},
        JPH::IndexedTriangle{27, 31, 30, kIndexMaterial},
        JPH::IndexedTriangle{27, 30, 26, kIndexMaterial},
    };
    vec.emplace_back(new JPH::MeshShapeSettings(std::move(list_vertex),
                     std::move(list_indexed_triangle)));
  }
  // outer_wall_segment_joined
  {
    JPH::VertexList list_vertex = {
        JPH::Float3{cast(0x426f9f80), cast(0x42208550), cast(0x40ccf10f)},
        JPH::Float3{cast(0x426f9f84), cast(0x42118550), cast(0x40ccf10d)},
        JPH::Float3{cast(0x42711ca6), cast(0x42118550), cast(0x40e1c719)},
        JPH::Float3{cast(0x42711ca1), cast(0x42208550), cast(0x40e1c719)},
        JPH::Float3{cast(0x4268fece), cast(0x4220854e), cast(0x40eb67b4)},
        JPH::Float3{cast(0x4268fed0), cast(0x4211854e), cast(0x40eb67ba)},
        JPH::Float3{cast(0x426a140c), cast(0x4211854e), cast(0x4100e4ba)},
        JPH::Float3{cast(0x426a140a), cast(0x4220854f), cast(0x4100e4b7)},
        JPH::Float3{cast(0x4264a788), cast(0x4220854e), cast(0x40f30c28)},
        JPH::Float3{cast(0x4264a788), cast(0x4211854e), cast(0x40f30c33)},
        JPH::Float3{cast(0x426512ff), cast(0x4211854e), cast(0x410567e0)},
        JPH::Float3{cast(0x426512ff), cast(0x4220854e), cast(0x410567db)},
        JPH::Float3{cast(0x42603119), cast(0x4220854e), cast(0x40f53a84)},
        JPH::Float3{cast(0x42603119), cast(0x4211854e), cast(0x40f53a91)},
        JPH::Float3{cast(0x42603119), cast(0x4211854e), cast(0x41069d48)},
        JPH::Float3{cast(0x42603119), cast(0x4220854e), cast(0x41069d42)},
        JPH::Float3{cast(0x425bbaab), cast(0x4220854e), cast(0x40f30c2a)},
        JPH::Float3{cast(0x425bbaaa), cast(0x4211854e), cast(0x40f30c35)},
        JPH::Float3{cast(0x425b4f33), cast(0x4211854e), cast(0x410567e1)},
        JPH::Float3{cast(0x425b4f33), cast(0x4220854e), cast(0x410567dc)},
        JPH::Float3{cast(0x42576364), cast(0x4220854e), cast(0x40eb67b4)},
        JPH::Float3{cast(0x42576362), cast(0x4211854e), cast(0x40eb67ba)},
        JPH::Float3{cast(0x42564e26), cast(0x4211854e), cast(0x4100e4ba)},
        JPH::Float3{cast(0x42564e28), cast(0x4220854e), cast(0x4100e4b7)},
        JPH::Float3{cast(0x4250c2b2), cast(0x42208550), cast(0x40ccf10f)},
        JPH::Float3{cast(0x4250c2ae), cast(0x42118550), cast(0x40ccf10d)},
        JPH::Float3{cast(0x424f458c), cast(0x42118550), cast(0x40e1c719)},
        JPH::Float3{cast(0x424f4590), cast(0x42208550), cast(0x40e1c719)},
        JPH::Float3{cast(0x420ac54a), cast(0x4213d21b), cast(0x0)},
        JPH::Float3{cast(0x420ac54a), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x42829d5b), cast(0x4213d21b), cast(0x0)},
        JPH::Float3{cast(0x42829d5b), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x42222c9a), cast(0x4213d21b), cast(0x0)},
        JPH::Float3{cast(0x42222c9a), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x426e4bb8), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x426e4bb8), cast(0x4213d21b), cast(0x0)},
        JPH::Float3{cast(0x423e3598), cast(0x4213d21b), cast(0x0)},
        JPH::Float3{cast(0x423e3598), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x425201c3), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x425201c3), cast(0x4213d21b), cast(0x0)},
        JPH::Float3{cast(0x420ac54a), cast(0x4213d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x42829d5b), cast(0x4213d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x42222c9a), cast(0x4213d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x426e4bb8), cast(0x4213d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x423e3598), cast(0x4213d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x425201c3), cast(0x4213d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x423e3598), cast(0x4217d21b), cast(0x0)},
        JPH::Float3{cast(0x42222c9a), cast(0x4217d21b), cast(0x0)},
        JPH::Float3{cast(0x426e4bb8), cast(0x4217d21b), cast(0x0)},
        JPH::Float3{cast(0x425201c3), cast(0x4217d21b), cast(0x0)},
        JPH::Float3{cast(0x42222c9a), cast(0x4217d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x426e4bb8), cast(0x4217d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x423e3598), cast(0x4217d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x425201c3), cast(0x4217d21b), cast(0x40e13a8e)},
        JPH::Float3{cast(0x4226d8c4), cast(0x4219e044), cast(0x40f65365)},
        JPH::Float3{cast(0x422b84ef), cast(0x421a9b58), cast(0x40feeba9)},
        JPH::Float3{cast(0x42303119), cast(0x421ad21b), cast(0x41009d47)},
        JPH::Float3{cast(0x4234dd44), cast(0x421a9b58), cast(0x40feeba9)},
        JPH::Float3{cast(0x4239896e), cast(0x4219e044), cast(0x40f65365)},
        JPH::Float3{cast(0x4226d8c4), cast(0x4219dd90), cast(0x0)},
        JPH::Float3{cast(0x422b84ef), cast(0x421a9b45), cast(0x0)},
        JPH::Float3{cast(0x42303119), cast(0x421ad21b), cast(0x0)},
        JPH::Float3{cast(0x4234dd44), cast(0x421a9b45), cast(0x0)},
        JPH::Float3{cast(0x4239896e), cast(0x4219dd90), cast(0x0)},
        JPH::Float3{cast(0x426994ba), cast(0x4219db44), cast(0x40f61127)},
        JPH::Float3{cast(0x4264ddbb), cast(0x421a9a51), cast(0x40fee072)},
        JPH::Float3{cast(0x426026bd), cast(0x421ad21b), cast(0x41009d47)},
        JPH::Float3{cast(0x425b6fbf), cast(0x421a9a51), cast(0x40fee072)},
        JPH::Float3{cast(0x4256b8c1), cast(0x4219db46), cast(0x40f6112c)},
        JPH::Float3{cast(0x426994ba), cast(0x4219d861), cast(0x0)},
        JPH::Float3{cast(0x4264ddbb), cast(0x421a9a3d), cast(0x0)},
        JPH::Float3{cast(0x426026bd), cast(0x421ad21b), cast(0x0)},
        JPH::Float3{cast(0x425b6fbf), cast(0x421a9a3d), cast(0x0)},
        JPH::Float3{cast(0x4256b8c1), cast(0x4219d862), cast(0x0)},
        JPH::Float3{cast(0x4226d8c4), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x422b84ef), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x42303119), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x4234dd44), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x4239896e), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x4239896e), cast(0x4213d21b), cast(0x40f69896)},
        JPH::Float3{cast(0x4234dd44), cast(0x4213d21b), cast(0x40feedf8)},
        JPH::Float3{cast(0x42303119), cast(0x4213d21b), cast(0x41009d47)},
        JPH::Float3{cast(0x422b84ef), cast(0x4213d21b), cast(0x40feedfa)},
        JPH::Float3{cast(0x4226d8c4), cast(0x4213d21b), cast(0x40f69896)},
        JPH::Float3{cast(0x4256b8c1), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x425b6fbf), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x426026bd), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x4264ddbb), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x426994ba), cast(0x4213d21b), cast(0x41a00000)},
        JPH::Float3{cast(0x426994ba), cast(0x4213d21b), cast(0x40f65a6a)},
        JPH::Float3{cast(0x4264ddbb), cast(0x4213d21b), cast(0x40fee2d8)},
        JPH::Float3{cast(0x426026bd), cast(0x4213d21b), cast(0x41009d47)},
        JPH::Float3{cast(0x425b6fbf), cast(0x4213d21b), cast(0x40fee2d8)},
        JPH::Float3{cast(0x4256b8c1), cast(0x4213d21b), cast(0x40f65a6c)},
        JPH::Float3{cast(0x423f9f80), cast(0x42208550), cast(0x40ccf10f)},
        JPH::Float3{cast(0x423f9f84), cast(0x42118550), cast(0x40ccf10d)},
        JPH::Float3{cast(0x42411ca6), cast(0x42118550), cast(0x40e1c719)},
        JPH::Float3{cast(0x42411ca1), cast(0x42208550), cast(0x40e1c719)},
        JPH::Float3{cast(0x4238fece), cast(0x4220854e), cast(0x40eb67b4)},
        JPH::Float3{cast(0x4238fed0), cast(0x4211854e), cast(0x40eb67ba)},
        JPH::Float3{cast(0x423a140c), cast(0x4211854e), cast(0x4100e4ba)},
        JPH::Float3{cast(0x423a140a), cast(0x4220854f), cast(0x4100e4b7)},
        JPH::Float3{cast(0x4234a788), cast(0x4220854e), cast(0x40f30c28)},
        JPH::Float3{cast(0x4234a788), cast(0x4211854e), cast(0x40f30c33)},
        JPH::Float3{cast(0x423512ff), cast(0x4211854e), cast(0x410567e0)},
        JPH::Float3{cast(0x423512ff), cast(0x4220854e), cast(0x410567db)},
        JPH::Float3{cast(0x42303119), cast(0x4220854e), cast(0x40f53a84)},
        JPH::Float3{cast(0x42303119), cast(0x4211854e), cast(0x40f53a91)},
        JPH::Float3{cast(0x42303119), cast(0x4211854e), cast(0x41069d48)},
        JPH::Float3{cast(0x42303119), cast(0x4220854e), cast(0x41069d42)},
        JPH::Float3{cast(0x422bbaab), cast(0x4220854e), cast(0x40f30c2a)},
        JPH::Float3{cast(0x422bbaaa), cast(0x4211854e), cast(0x40f30c35)},
        JPH::Float3{cast(0x422b4f33), cast(0x4211854e), cast(0x410567e1)},
        JPH::Float3{cast(0x422b4f33), cast(0x4220854e), cast(0x410567dc)},
        JPH::Float3{cast(0x42276364), cast(0x4220854e), cast(0x40eb67b4)},
        JPH::Float3{cast(0x42276362), cast(0x4211854e), cast(0x40eb67ba)},
        JPH::Float3{cast(0x42264e26), cast(0x4211854e), cast(0x4100e4ba)},
        JPH::Float3{cast(0x42264e28), cast(0x4220854e), cast(0x4100e4b7)},
        JPH::Float3{cast(0x4220c2b2), cast(0x42208550), cast(0x40ccf10f)},
        JPH::Float3{cast(0x4220c2ae), cast(0x42118550), cast(0x40ccf10d)},
        JPH::Float3{cast(0x421f458c), cast(0x42118550), cast(0x40e1c719)},
        JPH::Float3{cast(0x421f4590), cast(0x42208550), cast(0x40e1c719)},
    };
    JPH::IndexedTriangleList list_indexed_triangle = {
        JPH::IndexedTriangle{43, 41, 31, kIndexMaterial},
        JPH::IndexedTriangle{43, 31, 34, kIndexMaterial},
        JPH::IndexedTriangle{40, 42, 33, kIndexMaterial},
        JPH::IndexedTriangle{40, 33, 29, kIndexMaterial},
        JPH::IndexedTriangle{89, 43, 34, kIndexMaterial},
        JPH::IndexedTriangle{89, 34, 88, kIndexMaterial},
        JPH::IndexedTriangle{79, 44, 37, kIndexMaterial},
        JPH::IndexedTriangle{79, 37, 78, kIndexMaterial},
        JPH::IndexedTriangle{44, 45, 38, kIndexMaterial},
        JPH::IndexedTriangle{44, 38, 37, kIndexMaterial},
        JPH::IndexedTriangle{36, 39, 45, kIndexMaterial},
        JPH::IndexedTriangle{36, 45, 44, kIndexMaterial},
        JPH::IndexedTriangle{42, 32, 47, kIndexMaterial},
        JPH::IndexedTriangle{42, 47, 50, kIndexMaterial},
        JPH::IndexedTriangle{36, 44, 52, kIndexMaterial},
        JPH::IndexedTriangle{36, 52, 46, kIndexMaterial},
        JPH::IndexedTriangle{28, 32, 42, kIndexMaterial},
        JPH::IndexedTriangle{28, 42, 40, kIndexMaterial},
        JPH::IndexedTriangle{35, 30, 41, kIndexMaterial},
        JPH::IndexedTriangle{35, 41, 43, kIndexMaterial},
        JPH::IndexedTriangle{45, 39, 49, kIndexMaterial},
        JPH::IndexedTriangle{45, 49, 53, kIndexMaterial},
        JPH::IndexedTriangle{35, 43, 51, kIndexMaterial},
        JPH::IndexedTriangle{35, 51, 48, kIndexMaterial},
        JPH::IndexedTriangle{42, 83, 74, kIndexMaterial},
        JPH::IndexedTriangle{42, 74, 33, kIndexMaterial},
        JPH::IndexedTriangle{83, 82, 75, kIndexMaterial},
        JPH::IndexedTriangle{83, 75, 74, kIndexMaterial},
        JPH::IndexedTriangle{82, 81, 76, kIndexMaterial},
        JPH::IndexedTriangle{82, 76, 75, kIndexMaterial},
        JPH::IndexedTriangle{81, 80, 77, kIndexMaterial},
        JPH::IndexedTriangle{81, 77, 76, kIndexMaterial},
        JPH::IndexedTriangle{80, 79, 78, kIndexMaterial},
        JPH::IndexedTriangle{80, 78, 77, kIndexMaterial},
        JPH::IndexedTriangle{45, 93, 84, kIndexMaterial},
        JPH::IndexedTriangle{45, 84, 38, kIndexMaterial},
        JPH::IndexedTriangle{93, 92, 85, kIndexMaterial},
        JPH::IndexedTriangle{93, 85, 84, kIndexMaterial},
        JPH::IndexedTriangle{92, 91, 86, kIndexMaterial},
        JPH::IndexedTriangle{92, 86, 85, kIndexMaterial},
        JPH::IndexedTriangle{91, 90, 87, kIndexMaterial},
        JPH::IndexedTriangle{91, 87, 86, kIndexMaterial},
        JPH::IndexedTriangle{90, 89, 88, kIndexMaterial},
        JPH::IndexedTriangle{90, 88, 87, kIndexMaterial},
        JPH::IndexedTriangle{1, 5, 4, kIndexMaterial},
        JPH::IndexedTriangle{1, 4, 0, kIndexMaterial},
        JPH::IndexedTriangle{2, 6, 5, kIndexMaterial},
        JPH::IndexedTriangle{2, 5, 1, kIndexMaterial},
        JPH::IndexedTriangle{3, 7, 6, kIndexMaterial},
        JPH::IndexedTriangle{3, 6, 2, kIndexMaterial},
        JPH::IndexedTriangle{5, 9, 8, kIndexMaterial},
        JPH::IndexedTriangle{5, 8, 4, kIndexMaterial},
        JPH::IndexedTriangle{6, 10, 9, kIndexMaterial},
        JPH::IndexedTriangle{6, 9, 5, kIndexMaterial},
        JPH::IndexedTriangle{7, 11, 10, kIndexMaterial},
        JPH::IndexedTriangle{7, 10, 6, kIndexMaterial},
        JPH::IndexedTriangle{9, 13, 12, kIndexMaterial},
        JPH::IndexedTriangle{9, 12, 8, kIndexMaterial},
        JPH::IndexedTriangle{10, 14, 13, kIndexMaterial},
        JPH::IndexedTriangle{10, 13, 9, kIndexMaterial},
        JPH::IndexedTriangle{11, 15, 14, kIndexMaterial},
        JPH::IndexedTriangle{11, 14, 10, kIndexMaterial},
        JPH::IndexedTriangle{13, 17, 16, kIndexMaterial},
        JPH::IndexedTriangle{13, 16, 12, kIndexMaterial},
        JPH::IndexedTriangle{14, 18, 17, kIndexMaterial},
        JPH::IndexedTriangle{14, 17, 13, kIndexMaterial},
        JPH::IndexedTriangle{15, 19, 18, kIndexMaterial},
        JPH::IndexedTriangle{15, 18, 14, kIndexMaterial},
        JPH::IndexedTriangle{17, 21, 20, kIndexMaterial},
        JPH::IndexedTriangle{17, 20, 16, kIndexMaterial},
        JPH::IndexedTriangle{18, 22, 21, kIndexMaterial},
        JPH::IndexedTriangle{18, 21, 17, kIndexMaterial},
        JPH::IndexedTriangle{19, 23, 22, kIndexMaterial},
        JPH::IndexedTriangle{19, 22, 18, kIndexMaterial},
        JPH::IndexedTriangle{21, 25, 24, kIndexMaterial},
        JPH::IndexedTriangle{21, 24, 20, kIndexMaterial},
        JPH::IndexedTriangle{22, 26, 25, kIndexMaterial},
        JPH::IndexedTriangle{22, 25, 21, kIndexMaterial},
        JPH::IndexedTriangle{23, 27, 26, kIndexMaterial},
        JPH::IndexedTriangle{23, 26, 22, kIndexMaterial},
        JPH::IndexedTriangle{24, 25, 26, kIndexMaterial},
        JPH::IndexedTriangle{24, 26, 27, kIndexMaterial},
        JPH::IndexedTriangle{0, 3, 2, kIndexMaterial},
        JPH::IndexedTriangle{0, 2, 1, kIndexMaterial},
        JPH::IndexedTriangle{95, 99, 98, kIndexMaterial},
        JPH::IndexedTriangle{95, 98, 94, kIndexMaterial},
        JPH::IndexedTriangle{96, 100, 99, kIndexMaterial},
        JPH::IndexedTriangle{96, 99, 95, kIndexMaterial},
        JPH::IndexedTriangle{97, 101, 100, kIndexMaterial},
        JPH::IndexedTriangle{97, 100, 96, kIndexMaterial},
        JPH::IndexedTriangle{99, 103, 102, kIndexMaterial},
        JPH::IndexedTriangle{99, 102, 98, kIndexMaterial},
        JPH::IndexedTriangle{100, 104, 103, kIndexMaterial},
        JPH::IndexedTriangle{100, 103, 99, kIndexMaterial},
        JPH::IndexedTriangle{101, 105, 104, kIndexMaterial},
        JPH::IndexedTriangle{101, 104, 100, kIndexMaterial},
        JPH::IndexedTriangle{103, 107, 106, kIndexMaterial},
        JPH::IndexedTriangle{103, 106, 102, kIndexMaterial},
        JPH::IndexedTriangle{104, 108, 107, kIndexMaterial},
        JPH::IndexedTriangle{104, 107, 103, kIndexMaterial},
        JPH::IndexedTriangle{105, 109, 108, kIndexMaterial},
        JPH::IndexedTriangle{105, 108, 104, kIndexMaterial},
        JPH::IndexedTriangle{107, 111, 110, kIndexMaterial},
        JPH::IndexedTriangle{107, 110, 106, kIndexMaterial},
        JPH::IndexedTriangle{108, 112, 111, kIndexMaterial},
        JPH::IndexedTriangle{108, 111, 107, kIndexMaterial},
        JPH::IndexedTriangle{109, 113, 112, kIndexMaterial},
        JPH::IndexedTriangle{109, 112, 108, kIndexMaterial},
        JPH::IndexedTriangle{111, 115, 114, kIndexMaterial},
        JPH::IndexedTriangle{111, 114, 110, kIndexMaterial},
        JPH::IndexedTriangle{112, 116, 115, kIndexMaterial},
        JPH::IndexedTriangle{112, 115, 111, kIndexMaterial},
        JPH::IndexedTriangle{113, 117, 116, kIndexMaterial},
        JPH::IndexedTriangle{113, 116, 112, kIndexMaterial},
        JPH::IndexedTriangle{115, 119, 118, kIndexMaterial},
        JPH::IndexedTriangle{115, 118, 114, kIndexMaterial},
        JPH::IndexedTriangle{116, 120, 119, kIndexMaterial},
        JPH::IndexedTriangle{116, 119, 115, kIndexMaterial},
        JPH::IndexedTriangle{117, 121, 120, kIndexMaterial},
        JPH::IndexedTriangle{117, 120, 116, kIndexMaterial},
        JPH::IndexedTriangle{118, 119, 120, kIndexMaterial},
        JPH::IndexedTriangle{118, 120, 121, kIndexMaterial},
        JPH::IndexedTriangle{94, 97, 96, kIndexMaterial},
        JPH::IndexedTriangle{94, 96, 95, kIndexMaterial},
        JPH::IndexedTriangle{68, 53, 49, kIndexMaterial},
        JPH::IndexedTriangle{68, 49, 73, kIndexMaterial},
        JPH::IndexedTriangle{58, 63, 46, kIndexMaterial},
        JPH::IndexedTriangle{58, 46, 52, kIndexMaterial},
        JPH::IndexedTriangle{50, 47, 59, kIndexMaterial},
        JPH::IndexedTriangle{50, 59, 54, kIndexMaterial},
        JPH::IndexedTriangle{54, 59, 60, kIndexMaterial},
        JPH::IndexedTriangle{54, 60, 55, kIndexMaterial},
        JPH::IndexedTriangle{55, 60, 61, kIndexMaterial},
        JPH::IndexedTriangle{55, 61, 56, kIndexMaterial},
        JPH::IndexedTriangle{56, 61, 62, kIndexMaterial},
        JPH::IndexedTriangle{56, 62, 57, kIndexMaterial},
        JPH::IndexedTriangle{57, 62, 63, kIndexMaterial},
        JPH::IndexedTriangle{57, 63, 58, kIndexMaterial},
        JPH::IndexedTriangle{51, 64, 69, kIndexMaterial},
        JPH::IndexedTriangle{51, 69, 48, kIndexMaterial},
        JPH::IndexedTriangle{64, 65, 70, kIndexMaterial},
        JPH::IndexedTriangle{64, 70, 69, kIndexMaterial},
        JPH::IndexedTriangle{65, 66, 71, kIndexMaterial},
        JPH::IndexedTriangle{65, 71, 70, kIndexMaterial},
        JPH::IndexedTriangle{66, 67, 72, kIndexMaterial},
        JPH::IndexedTriangle{66, 72, 71, kIndexMaterial},
        JPH::IndexedTriangle{67, 68, 73, kIndexMaterial},
        JPH::IndexedTriangle{67, 73, 72, kIndexMaterial},
    };
    vec.emplace_back(new JPH::MeshShapeSettings(std::move(list_vertex),
                     std::move(list_indexed_triangle)));
  }

  return vec;
}

// ============= Machinery to run test (basically HelloWorld)

// Helpers

static constexpr JPH::BroadPhaseLayer::Type bp_value(JPH::BroadPhaseLayer bp) {
  return static_cast<JPH::BroadPhaseLayer::Type>(bp);
}

// Layers and collision rules

namespace ObjectLayerImpl {
static constexpr JPH::ObjectLayer kStatic = 0;
static constexpr JPH::ObjectLayer kDynamic = 1;
[[maybe_unused]] static constexpr JPH::uint kCount = 2;
};  // namespace ObjectLayerImpl

namespace BroadPhaseLayerImpl {
static constexpr JPH::BroadPhaseLayer kStatic{0};
static constexpr JPH::BroadPhaseLayer kDynamic{1};
static constexpr JPH::uint kCount = 2;
};  // namespace BroadPhaseLayerImpl

const char* to_string(JPH::BroadPhaseLayer bp) {
  switch (bp_value(bp)) {
    case bp_value(BroadPhaseLayerImpl::kStatic):
      return "BroadPhaseLayerImpl::kStatic";
    case bp_value(BroadPhaseLayerImpl::kDynamic):
      return "BroadPhaseLayerImpl::kDynamic";
    default:
      JPH_ASSERT(false);
      return "BroadPhaseLayerImpl::__unknown__";
  }
}

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
 public:
  virtual bool ShouldCollide(JPH::ObjectLayer inLayer1,
                             JPH::ObjectLayer inLayer2) const override {
    switch (inLayer1) {
      case ObjectLayerImpl::kStatic:
        return inLayer2 == ObjectLayerImpl::kDynamic;
      case ObjectLayerImpl::kDynamic:
        return true;
      default:
        JPH_ASSERT(false);
        return false;
    }
  }
};

class BroadPhaseLayerInterfaceImpl : public JPH::BroadPhaseLayerInterface {
 public:
  virtual JPH::uint GetNumBroadPhaseLayers() const override {
    return BroadPhaseLayerImpl::kCount;
  }

  virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(
      JPH::ObjectLayer inLayer) const override {
    switch (inLayer) {
      case ObjectLayerImpl::kStatic:
        return BroadPhaseLayerImpl::kStatic;
      case ObjectLayerImpl::kDynamic:
        return BroadPhaseLayerImpl::kDynamic;
      default:
        JPH_ASSERT(false);
        return BroadPhaseLayerImpl::kStatic;
    }
  }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  virtual const char* GetBroadPhaseLayerName(
      JPH::BroadPhaseLayer inLayer) const override {
    return to_string(inLayer);
  }
#endif  // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED
};

class ObjectVsBroadPhaseLayerFilterImpl
    : public JPH::ObjectVsBroadPhaseLayerFilter {
 public:
  virtual bool ShouldCollide(JPH::ObjectLayer inLayer1,
                             JPH::BroadPhaseLayer inLayer2) const override {
    switch (inLayer1) {
      case ObjectLayerImpl::kStatic:
        return inLayer2 == BroadPhaseLayerImpl::kDynamic;
      case ObjectLayerImpl::kDynamic:
        return true;
      default:
        JPH_ASSERT(false);
        return false;
    }
  }
};

// Main logic

int main() {
  // Set up persistent state
  JPH::RegisterDefaultAllocator();
  JPH::Factory::sInstance = new JPH::Factory();
  JPH::RegisterTypes();

  // Resources used during physics update
  JPH::TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
  JPH::JobSystemSingleThreaded job_system(JPH::cMaxPhysicsJobs);

  // Physics system settings
  const JPH::uint kMaxBodies = 1024;
  const JPH::uint kNumBodyMutexes = 0;  // let impl auto-detect
  const JPH::uint kMaxBodyPairs = 1024;
  const JPH::uint kMaxContactConstraints = 1024;
  BroadPhaseLayerInterfaceImpl bpli_impl;
  ObjectVsBroadPhaseLayerFilterImpl ovbplf_impl;
  ObjectLayerPairFilterImpl olpf_impl;

  // Create physics system
  JPH::PhysicsSystem physics_system;
  physics_system.SetGravity(test_physics_system_gravity());
  physics_system.Init(kMaxBodies, kNumBodyMutexes, kMaxBodyPairs,
                      kMaxContactConstraints, bpli_impl, ovbplf_impl,
                      olpf_impl);

  // Add bodies
  JPH::BodyInterface& body_interface = physics_system.GetBodyInterface();
  std::vector<JPH::BodyID> vec_id_body;
  {
    auto vec_settings = test_vec_mesh_shape_settings();
    const JPH::RVec3 p_world(0.0f, 0.0f, 0.0f);
    for (const auto& settings : vec_settings) {
      JPH::BodyCreationSettings body_settings(
          settings->Create().Get(), p_world, JPH::Quat::sIdentity(),
          JPH::EMotionType::Static, ObjectLayerImpl::kStatic);
      vec_id_body.emplace_back(body_interface.CreateAndAddBody(
          body_settings, JPH::EActivation::DontActivate));
    }
  }

  // Finish adding bodies
  physics_system.OptimizeBroadPhase();

  // Set up controller
  JPH::Ref<JPH::CharacterVirtual> character_virtual;
  {
    auto settings = test_character_virtual_settings();
    character_virtual = new JPH::CharacterVirtual(
        settings, test_character_position_initial(), JPH::Quat::sIdentity(),
        &physics_system);
  }

  // Run simulation for a while
  JPH::Vec3 p_last = character_virtual->GetPosition();
  float max_length_delta = 0.0f;
  const size_t kMaxNumSteps = 100;
  for (size_t num_steps = 0; num_steps < kMaxNumSteps; num_steps++) {
    const auto p_this = character_virtual->GetPosition();
    const auto delta = p_this - p_last;
    p_last = p_this;
    const float length_delta = delta.Length();
    if (length_delta > max_length_delta) {
      max_length_delta = length_delta;
    }
    const char* prefix = length_delta > 0.3f ? ">" : " ";
    std::cout << prefix << " pos.xy: (" << p_this.GetX() << ", "
              << p_this.GetY() << ")" << " delta.xy: (" << delta.GetX() << ", "
              << delta.GetY() << ")" << std::endl;

    const float delta_time = test_delta_time();
    test_character_set_linear_velocity(character_virtual, delta_time);
    character_virtual->ExtendedUpdate(
        delta_time, physics_system.GetGravity(),
        test_extended_update_settings(),
        physics_system.GetDefaultBroadPhaseLayerFilter(
            ObjectLayerImpl::kDynamic),
        physics_system.GetDefaultLayerFilter(ObjectLayerImpl::kDynamic), {},
        {}, temp_allocator);

    const JPH::uint kCollisionSteps = 1;
    physics_system.Update(delta_time, kCollisionSteps, &temp_allocator,
                          &job_system);
  }
  std::cout << std::endl << "max delta: " << max_length_delta << std::endl;

  // Tear down controller -- handled by Ref*

  // Remove/destroy bodies
  for (const auto id_body : vec_id_body) {
    body_interface.RemoveBody(id_body);
    body_interface.DestroyBody(id_body);
  }

  // Tear down persistent state
  JPH::UnregisterTypes();
  delete JPH::Factory::sInstance;
  JPH::Factory::sInstance = nullptr;
}
