# -*- encoding: utf-8 -*-
# stub: jwt 2.2.1 ruby lib

Gem::Specification.new do |s|
  s.name = "jwt"
  s.version = "2.2.1"

  s.required_rubygems_version = Gem::Requirement.new(">= 0") if s.respond_to? :required_rubygems_version=
  s.require_paths = ["lib"]
  s.authors = ["Tim Rudat"]
  s.date = "2019-05-24"
  s.description = "A pure ruby implementation of the RFC 7519 OAuth JSON Web Token (JWT) standard."
  s.email = "timrudat@gmail.com"
  s.homepage = "https://github.com/jwt/ruby-jwt"
  s.licenses = ["MIT"]
  s.required_ruby_version = Gem::Requirement.new(">= 2.1")
  s.rubygems_version = "2.5.2.1"
  s.summary = "JSON Web Token implementation in Ruby"

  s.installed_by_version = "2.5.2.1" if s.respond_to? :installed_by_version

  if s.respond_to? :specification_version then
    s.specification_version = 4

    if Gem::Version.new(Gem::VERSION) >= Gem::Version.new('1.2.0') then
      s.add_development_dependency(%q<appraisal>, [">= 0"])
      s.add_development_dependency(%q<bundler>, [">= 0"])
      s.add_development_dependency(%q<rake>, [">= 0"])
      s.add_development_dependency(%q<rspec>, [">= 0"])
      s.add_development_dependency(%q<simplecov>, [">= 0"])
      s.add_development_dependency(%q<simplecov-json>, [">= 0"])
      s.add_development_dependency(%q<codeclimate-test-reporter>, [">= 0"])
      s.add_development_dependency(%q<codacy-coverage>, [">= 0"])
      s.add_development_dependency(%q<rbnacl>, [">= 0"])
      s.add_development_dependency(%q<openssl>, ["~> 2.1"])
    else
      s.add_dependency(%q<appraisal>, [">= 0"])
      s.add_dependency(%q<bundler>, [">= 0"])
      s.add_dependency(%q<rake>, [">= 0"])
      s.add_dependency(%q<rspec>, [">= 0"])
      s.add_dependency(%q<simplecov>, [">= 0"])
      s.add_dependency(%q<simplecov-json>, [">= 0"])
      s.add_dependency(%q<codeclimate-test-reporter>, [">= 0"])
      s.add_dependency(%q<codacy-coverage>, [">= 0"])
      s.add_dependency(%q<rbnacl>, [">= 0"])
      s.add_dependency(%q<openssl>, ["~> 2.1"])
    end
  else
    s.add_dependency(%q<appraisal>, [">= 0"])
    s.add_dependency(%q<bundler>, [">= 0"])
    s.add_dependency(%q<rake>, [">= 0"])
    s.add_dependency(%q<rspec>, [">= 0"])
    s.add_dependency(%q<simplecov>, [">= 0"])
    s.add_dependency(%q<simplecov-json>, [">= 0"])
    s.add_dependency(%q<codeclimate-test-reporter>, [">= 0"])
    s.add_dependency(%q<codacy-coverage>, [">= 0"])
    s.add_dependency(%q<rbnacl>, [">= 0"])
    s.add_dependency(%q<openssl>, ["~> 2.1"])
  end
end
