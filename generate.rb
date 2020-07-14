class Container
  attr_reader :service_name, :container_name, :volume_name, :id

  def initialize(name, id)
    @service_name = "tb3-ros-#{name}"
    @container_name = "tb3-ros-#{name}"
    @volume_name = "rospersistent-#{name}"
    @id = id
  end

  def service
    <<-YAML
  #{@service_name}:
    container_name: #{@container_name}
    image: cosi119/tb3-ros:latest
    ports:
      - '#{8080 + @id}:8080'
      - '#{6080 + @id}:6080'
      - '#{2220 + @id}:22'
    volumes:
      - type: volume
        source: #{@volume_name}
        target: /my_ros_data
        volume:
          nocopy: false
    YAML
  end

  def volume
    <<-YAML
  #{@volume_name}:
    driver_opts:
      type: none
      device: ROSPERSISTENT-PATH/data/#{@volume_name}
      o: bind
    YAML
  end
end

# Generate
i = 0
containers = ARGV.map do |n|
  i = i + 1
  Container.new(n, i)
end

# Generate docker-compose
File.open("docker-compose-server.yaml", 'w') do |f|
  body1 = <<~YAML
  version: '3.7'
  services:
  YAML

  body2 = <<~YAML
  volumes:
  YAML

  f.write(body1)
  containers.each { |c| f.write(c.service) }
  f.write(body2)
  containers.each { |c| f.write(c.volume) }
end

p "Add the followings to Makefile"
containers.each { |c| p "@[ -d data/#{c.volume_name} ] || mkdir -p data/#{c.volume_name}" }
