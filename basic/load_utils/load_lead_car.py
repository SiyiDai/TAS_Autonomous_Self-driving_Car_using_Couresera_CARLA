def load_lead_car(measurement_data):
    # Obtain Lead Vehicle information.
    lead_car_pos = []
    lead_car_length = []
    lead_car_speed = []
    for agent in measurement_data.non_player_agents:
        agent_id = agent.id
        if agent.HasField("vehicle"):
            lead_car_pos.append(
                [
                    agent.vehicle.transform.location.x,
                    agent.vehicle.transform.location.y,
                ]
            )
            lead_car_length.append(agent.vehicle.bounding_box.extent.x)
            lead_car_speed.append(agent.vehicle.forward_speed)
    return lead_car_pos, lead_car_length, lead_car_speed
