# -*- encoding: utf-8 -*-
# stub: oauth2 1.4.1 ruby lib

Gem::Specification.new do |s|
  s.name = "oauth2"
  s.version = "1.4.1"

  s.required_rubygems_version = Gem::Requirement.new(">= 1.3.5") if s.respond_to? :required_rubygems_version=
  s.require_paths = ["lib"]
  s.authors = ["Peter Boling", "Michael Bleigh", "Erik Michaels-Ober"]
  s.bindir = "exe"
  s.date = "2018-10-13"
  s.description = "A Ruby wrapper for the OAuth 2.0 protocol built with a similar style to the original OAuth spec."
  s.email = ["peter.boling@gmail.com"]
  s.homepage = "https://github.com/oauth-xx/oauth2"
  s.licenses = ["MIT"]
  s.required_ruby_version = Gem::Requirement.new(">= 1.9.0")
  s.rubygems_version = "2.5.2.1"
  s.summary = "A Ruby wrapper for the OAuth 2.0 protocol."

  s.installed_by_version = "2.5.2.1" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4

    if Gem::Version.new(Gem::VERSION) >= Gem::Version.new('1.2.0') then
      s.add_runtime_dependency(%q<faraday>, ["< 0.16.0", ">= 0.8"])
      s.add_runtime_dependency(%q<jwt>, ["< 3.0", ">= 1.0"])
      s.add_runtime_dependency(%q<multi_json>, ["~> 1.3"])
      s.add_runtime_dependency(%q<multi_xml>, ["~> 0.5"])
      s.add_runtime_dependency(%q<rack>, ["< 3", ">= 1.2"])
      s.add_development_dependency(%q<addressable>, ["~> 2.3"])
      s.add_development_dependency(%q<backports>, ["~> 3.11"])
      s.add_development_dependency(%q<bundler>, ["~> 1.16"])
      s.add_development_dependency(%q<coveralls>, ["~> 0.8"])
      s.add_development_dependency(%q<rake>, ["~> 12.3"])
      s.add_development_dependency(%q<rdoc>, ["< 7", ">= 5.0"])
      s.add_development_dependency(%q<rspec>, ["~> 3.0"])
      s.add_development_dependency(%q<wwtd>, [">= 0"])
    else
      s.add_dependency(%q<faraday>, ["< 0.16.0", ">= 0.8"])
      s.add_dependency(%q<jwt>, ["< 3.0", ">= 1.0"])
      s.add_dependency(%q<multi_json>, ["~> 1.3"])
      s.add_dependency(%q<multi_xml>, ["~> 0.5"])
      s.add_dependency(%q<rack>, ["< 3", ">= 1.2"])
      s.add_dependency(%q<addressable>, ["~> 2.3"])
      s.add_dependency(%q<backports>, ["~> 3.11"])
      s.add_dependency(%q<bundler>, ["~> 1.16"])
      s.add_dependency(%q<coveralls>, ["~> 0.8"])
      s.add_dependency(%q<rake>, ["~> 12.3"])
      s.add_dependency(%q<rdoc>, ["< 7", ">= 5.0"])
      s.add_dependency(%q<rspec>, ["~> 3.0"])
      s.add_dependency(%q<wwtd>, [">= 0"])
    end
  else
    s.add_dependency(%q<faraday>, ["< 0.16.0", ">= 0.8"])
    s.add_dependency(%q<jwt>, ["< 3.0", ">= 1.0"])
    s.add_dependency(%q<multi_json>, ["~> 1.3"])
    s.add_dependency(%q<multi_xml>, ["~> 0.5"])
    s.add_dependency(%q<rack>, ["< 3", ">= 1.2"])
    s.add_dependency(%q<addressable>, ["~> 2.3"])
    s.add_dependency(%q<backports>, ["~> 3.11"])
    s.add_dependency(%q<bundler>, ["~> 1.16"])
    s.add_dependency(%q<coveralls>, ["~> 0.8"])
    s.add_dependency(%q<rake>, ["~> 12.3"])
    s.add_dependency(%q<rdoc>, ["< 7", ">= 5.0"])
    s.add_dependency(%q<rspec>, ["~> 3.0"])
    s.add_dependency(%q<wwtd>, [">= 0"])
  end
end
