extern crate nalgebra as na;

use na::{Point3, RealField, Vector3};
use ncollide3d::shape::{Ball, Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::object::{
    BodyHandle, BodyPartHandle, BodySet, ColliderDesc, ColliderSet, DefaultBodyHandle,
    DefaultBodySet, DefaultColliderHandle, DefaultColliderSet, Ground, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::Testbed;

fn main() {
    let mut mechanical_world = DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0));
    let mut geometrical_world = DefaultGeometricalWorld::new();

    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let mut joint_constraints = DefaultJointConstraintSet::new();
    let mut force_generators = DefaultForceGeneratorSet::new();

    let ground_handle = add_ground(&mut bodies, &mut colliders);
    let object_handle = add_ball(&mut bodies, &mut colliders);

    // Provide to the testbed all the components of our physics simulation.
    let mut testbed = Testbed::new(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    // Tell the testbed your ground has a handle equal to `ground_handle`.
    // This will make it be drawn in gray.
    testbed.set_ground_handle(Some(ground_handle));
    // Adjust the initial camera pose.
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));

    testbed.run();
}

fn add_ground(
    bodies: &mut DefaultBodySet<f32>,
    colliders: &mut DefaultColliderSet<f32>,
) -> DefaultBodyHandle {
    let ground_thickness = 0.2;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(3.0, ground_thickness, 3.0)));

    // Build a static ground body and add it to the body set.
    let ground_handle = bodies.insert(Ground::new());

    // Build the collider.
    let co = ColliderDesc::new(ground_shape)
        .translation(Vector3::y() * -ground_thickness)
        .material(MaterialHandle::new(BasicMaterial::new(1.0, 0.8)))
        .build(BodyPartHandle(ground_handle, 0));
    // Add the collider to the collider set.
    colliders.insert(co);
    ground_handle
}

fn add_ball(
    bodies: &mut DefaultBodySet<f32>,
    colliders: &mut DefaultColliderSet<f32>,
) -> DefaultColliderHandle {
    // Build the rigid body.
    let rb = RigidBodyDesc::new()
        .translation(Vector3::new(0.0, 3.0, 0.0))
        .build();
    // Insert the rigid body to the body set.
    let rb_handle = bodies.insert(rb);

    // Build the collider.
    let ball = ShapeHandle::new(Ball::new(0.1));
    let co = ColliderDesc::new(ball.clone())
        .density(1.0)
        .material(MaterialHandle::new(BasicMaterial::new(0.9, 0.8)))
        .build(BodyPartHandle(rb_handle, 0));
    // Insert the collider to the body set.
    colliders.insert(co)
}
