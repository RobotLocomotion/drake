#include "drake/examples/box/two_boxes_plant.h"
#include "drake/common/eigen_types.h"
#include "drake/common/default_scalars.h"
#include "drake/examples/box/box_geometry.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/examples/box/spring_plant.h"
namespace drake{
namespace examples{
namespace box{

    /// Constructs a default plant.
    template <typename T> 
    TwoBoxesPlant<T>::TwoBoxesPlant() : TwoBoxesPlant( (1.0) /* box m */, (0.0) /* box vis */, 
                                                (1.0) /* box l */, (1.0) /* sk */, (0.0) /* sd */ )
    {
    }
    /// constructs a plant with box mass (m), box visc (d), box length (l),
    ///               spring stiffness (sk), spring damping (sd)
    template <typename T> 
    TwoBoxesPlant<T>::TwoBoxesPlant(double m, double d, double l, double sk, double sd) : 
      systems::Diagram<T>(systems::SystemTypeTag<box::TwoBoxesPlant>{}), m_(m), d_(d), l_(l), sk_(sk), sd_(sd)
    {
        systems::DiagramBuilder<T> builder;

        pBox1_ = builder.template AddSystem<BoxPlant<T>>((1.0) / m, l, d);
        pBox1_->set_name("box1");

        pBox2_ = builder.template AddSystem<BoxPlant<T>>((1.0) / m, l, d);
        pBox2_->set_name("box2");

        auto pSpring = builder.template AddSystem<SpringPlant<T>>(sk, sd, l);
        pSpring->set_name("spring");

        auto pAdder1 = builder.template AddSystem<systems::Adder<T>>(2, 1); // 2 input, 1 output
        auto pAdder2 = builder.template AddSystem<systems::Adder<T>>(1, 1);
        auto pNegater = builder.template AddSystem<systems::Gain<T>>(-1., 1);

        auto pMultiplexer = builder.template AddSystem<systems::Multiplexer<T>>(std::vector<int>{2,2,1});

        // connect spring inputs
        builder.Connect(pBox1_->get_output_port(), pSpring->get_first_box_input_port());
        builder.Connect(pBox2_->get_output_port(), pSpring->get_second_box_input_port());

        // use spring's output in the positive for box 2
        //builder.Connect(source2->get_output_port(), adder2->get_input_port(1));
        builder.Connect(pSpring->get_force_output_port(), pAdder2->get_input_port(0));
        builder.Connect(pAdder2->get_output_port(), pBox2_->get_input_port());

        // use spring's output in the negative for box 1
        builder.Connect(pSpring->get_force_output_port(), pNegater->get_input_port() );
        builder.Connect(pNegater->get_output_port(), pAdder1->get_input_port(0) );
        builder.Connect(pAdder1->get_output_port(), pBox1_->get_input_port());

        // combine states and force into one output port for logger
        builder.Connect(pBox1_->get_output_port(), pMultiplexer->get_input_port(0));
        builder.Connect(pBox2_->get_output_port(), pMultiplexer->get_input_port(1));
        builder.Connect(pSpring->get_force_output_port(), pMultiplexer->get_input_port(2));

        // export the 1D force on box1
        builder.ExportInput(pAdder1->get_input_port(1));

        box1output_ = builder.ExportOutput(pBox1_->get_state_output_port());
        box2output_ = builder.ExportOutput(pBox2_->get_state_output_port());
        logport_ = builder.ExportOutput(pMultiplexer->get_output_port(0));

        builder.BuildInto(this);
    }

    /// pass in length-4 vec
    template <typename T> 
    void TwoBoxesPlant<T>::SetInitialStates(systems::Context<T>* context,
                            const drake::VectorX<T>& initState)
    {
        DRAKE_DEMAND(initState.rows() * initState.cols() == 4);
        context->get_mutable_continuous_state_vector().SetFromVector(initState);
    }

    /// pass in length-2 vec for the states
    template <typename T> 
    void TwoBoxesPlant<T>::SetBox1State(systems::Context<T>* context, const drake::VectorX<T>& state)
    {
        DRAKE_DEMAND(state.rows() * state.cols() == 2);
        pBox1_->set_initial_state(&(this->GetMutableBox1Context(context)), state);
    }
    

    template <typename T> 
    void TwoBoxesPlant<T>::SetBox2State(systems::Context<T>* context, const drake::VectorX<T>& state)
    {
        DRAKE_DEMAND(state.rows() * state.cols() == 2);
        pBox2_->set_initial_state(&(this->GetMutableBox2Context(context)), state);
    }
    
    template <typename T> 
    drake::VectorX<T> TwoBoxesPlant<T>::GetState(const systems::Context<T>& context) const
    {
        return context.get_continuous_state_vector().CopyToVector();
    }
    
    template <typename T> 
    drake::VectorX<T> TwoBoxesPlant<T>::GetBox1State(const systems::Context<T>& context) const
    {
    
        return pBox1_->GetBoxState(this->GetBox1Context(context));
    }
    
    template <typename T> 
    drake::VectorX<T> TwoBoxesPlant<T>::GetBox2State(const systems::Context<T>& context) const
    {
        return pBox2_->GetBoxState(this->GetBox2Context(context));
    }
    
    template <typename T> 
    T TwoBoxesPlant<T>::GetBox1Position(const systems::Context<T>& context) const
    {
        return  (pBox1_->GetBoxState(this->GetBox1Context(context)))(0);
    }

    template <typename T> 
    T TwoBoxesPlant<T>::GetBox1Velocity(const systems::Context<T>& context) const
    {
        return  (pBox1_->GetBoxState(this->GetBox1Context(context)))(1);
    }

    template <typename T> 
    T TwoBoxesPlant<T>::GetBox2Position(const systems::Context<T>& context) const
    {
        return (pBox2_->GetBoxState(this->GetBox2Context(context)))(1);
    }

    template <typename T> 
    T TwoBoxesPlant<T>::GetBox2Velocity(const systems::Context<T>& context) const
    {
        return (pBox2_->GetBoxState(this->GetBox2Context(context)))(1);
    }

    template <typename T> 
    systems::Context<T>&  TwoBoxesPlant<T>::GetMutableBox1Context(systems::Context<T>* context)
    {
        return this->GetMutableSubsystemContext( *pBox1_, context);
    }

    template <typename T> 
    systems::Context<T>&  TwoBoxesPlant<T>::GetMutableBox2Context(systems::Context<T>* context)
    {
        return this->GetMutableSubsystemContext( *pBox2_, context);
    }

    template <typename T> 
    const systems::Context<T>&  TwoBoxesPlant<T>::GetBox1Context(const systems::Context<T>& context) const
    {
        return this->GetSubsystemContext( *pBox1_, context);
    }

    template <typename T> 
    const systems::Context<T>&  TwoBoxesPlant<T>::GetBox2Context(const systems::Context<T>& context) const
    {
        return this->GetSubsystemContext( *pBox2_, context);
    }

    template <typename T>
    T TwoBoxesPlant<T>::CalcBoxesTotalEnergy(const systems::Context<T>& context) const
    {
        const systems::Context<T>& box1context = this->GetBox1Context(context);
        const systems::Context<T>& box2context = this->GetBox2Context(context);
        return pBox1_->CalcTotalEnergy(box1context) + pBox2_->CalcTotalEnergy(box2context);
    }
    void AddGeometryToBuilder(systems::DiagramBuilder<double>* builder, 
                          const TwoBoxesPlant<double>& plant, 
                          geometry::SceneGraph<double>* scene_graph)
    {
        BoxGeometry::AddToBuilder(
            builder, plant.Box1(), plant.get_box1_output(), scene_graph, "1");
        BoxGeometry::AddToBuilder(
            builder, plant.Box2(), plant.get_box2_output(), scene_graph, "2");
    }
} // box
} // examples
} // drake


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(class ::drake::examples::box::TwoBoxesPlant)