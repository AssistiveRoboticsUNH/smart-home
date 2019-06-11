# -*- encoding: utf-8 -*-
# stub: rack-protection 2.0.5 ruby lib

Gem::Specification.new do |s|
  s.name = "rack-protection"
  s.version = "2.0.5"

  s.required_rubygems_version = Gem::Requirement.new(">= 0") if s.respond_to? :required_rubygems_version=
  s.metadata = { "documentation_uri" => "https://www.rubydoc.info/gems/rack-protection", "homepage_uri" => "http://sinatrarb.com/protection/", "source_code_uri" => "https://github.com/sinatra/sinatra/tree/master/rack-protection" } if s.respond_to? :metadata=
  s.require_paths = ["lib"]
  s.authors = ["https://github.com/sinatra/sinatra/graphs/contributors"]
  s.date = "2018-12-22"
  s.description = "Protect against typical web attacks, works with all Rack apps, including Rails."
  s.email = "sinatrarb@googlegroups.com"
  s.homepage = "http://sinatrarb.com/protection/"
  s.licenses = ["MIT"]
  s.rubygems_version = "2.5.2.1"
  s.summary = "Protect against typical web attacks, works with all Rack apps, including Rails."

  s.installed_by_version = "2.5.2.1" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4

    if Gem::Version.new(Gem::VERSION) >= Gem::Version.new('1.2.0') then
      s.add_runtime_dependency(%q<rack>, [">= 0"])
      s.add_development_dependency(%q<rack-test>, [">= 0"])
      s.add_development_dependency(%q<rspec>, ["~> 3.6"])
    else
      s.add_dependency(%q<rack>, [">= 0"])
      s.add_dependency(%q<rack-test>, [">= 0"])
      s.add_dependency(%q<rspec>, ["~> 3.6"])
    end
  else
    s.add_dependency(%q<rack>, [">= 0"])
    s.add_dependency(%q<rack-test>, [">= 0"])
    s.add_dependency(%q<rspec>, ["~> 3.6"])
  end
end
